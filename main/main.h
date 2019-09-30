//
// Created by drill on 9/5/2019.
//

#ifndef TRYING_MAIN_H
#define TRYING_MAIN_H

#include <thread>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <ctime>
#include <string>
#include <cmath>
#include <csignal>
#include <sys/types.h>
#include <sys/mman.h>
#include <cinttypes>
#include <mutex>
#include <bitset>
#include <chrono>
#include <fstream>
#include <vector>
#include "Eigen/Dense"

extern "C"{
#include <rc/math/filter.h>
}

#include "fpga_headers/hps_0.h"
#include "fpga_headers/hwlib.h"
#include "fpga_headers/socal.h"
#include "fpga_headers/hps.h"
#include "fpga_headers/alt_gpio.h"
#include "DE0_Motor_Controller/motorController.h"
#include "DE0_Motor_Controller/fpgaAddressComm.h"
#include "DE0_Motor_Controller/fpgaEncoder.h"
#include "DE0_Motor_Controller/AS5048b.h"
#include "INIReader.h"

//#define P_float 0.0005
//#define I_float 0.00005
//#define D_float 0.0000006
//#define SAMPLE_RATE 1000
//#define SYNC_TOLERANCE 10
//#define dt (1.0/(float)SAMPLE_RATE)
//#define interval_time_us ((int)(dt * 1000000))
//#define update_dt 0.01
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )
//#define MAX_TRAVEL_RANGE 10000000
//#define MAX_CURRENT 2.5 //amps


//PID
float P_float, I_float, D_float;
uint16_t sampleRate;
uint8_t  reload_flag = 0;
uint8_t  exit_flag_main = 0;
//--------------------------------------------------
//------------------global variables------------------------
int32_t setPosMBack(0); int32_t setPosMInsertion(0); int32_t setPosMTip(0); int32_t setPosMMid(0);

//addresses
unsigned char* h2p_lw_gpio0_addr;
unsigned char* h2p_lw_heartbeat_addr;
unsigned char* h2p_lw_quad_reset_addr;
unsigned char* h2p_lw_quad_addr[8];
unsigned char* h2p_lw_pwm_values_addr[8];
unsigned char* h2p_lw_adc;
unsigned char* h2p_lw_gpio1_addr;

int CURRENT_FLAG;
int TRAVEL_FLAG;
int ETSOP_FLAG;
int exit_flag;
double dt = 0.001;

int fd;
void* virtual_base;

std::mutex mtx;
//-------------------------------------------------------------
//safety limits
int16_t InsertionJointLimitUpper,
        InsertionJointLimitLower,
        TipJointLimitUpper,
        TipJointLimitLower,
        MidJointLimitUpper,
        MidJointLimitLower,
        BackJointLimitUpper,
        BackJointLimitLower,
        RotationJointLimitUpper,
        RotationJointLimitLower,
        motorBackOffset,
        motorTipOffset,
        motorMidOffset,
        PWMCap;

int32_t LinearAxisUpperLimit,
        LinearAxisLowerLimit,
        VerticalAxisUpperLimit,
        VerticalAxisLowerLimit,
        HorizontalAxisUpperLimit,
        HorizontalAxisLowerLimit;

float     InsertionMotorCurrentLimit,
        TipMotorCurrentLimit,
        MidMotorCurrentLimit,
        BackMotorCurrentLimit,
        RotationMotorCurrentLimit,
        LinearAxisMotorCurrentLimit,
        VerticalAxisMotorCurrentLimit,
        HorizontalAxisMotorCurrentLimit;

//----------------

std::vector<CTRobot::motorController> motorStack;
//CTRobot::motorController motorBack;
//CTRobot::motorController motorInsertion;
//CTRobot::motorController motorTip;
//CTRobot::motorController motorMid;
//CTRobot::motorController motorRotation;
//CTRobot::motorController motorLinear;
//CTRobot::motorController motorVertical;
//CTRobot::motorController motorHorizontal;

//-----i2c sensor bus--------------------------
std::string busName = "/dev/i2c-1";
auto *theBus = new CTRobot::I2CBus(busName);
//---------------------------------------------

//helper functions
void signalHandler(int sigNum){
    exit_flag = 1;
    exit_flag_main = 1;
    std::cout << "setting exit_flag to 1" << std:: endl;
}

void motorAddressesSetup(){
    exit_flag = 0;
//initialization shared memory
    if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
        throw std::runtime_error(std::string("could not open /dev/mem"));
    }
    virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );
    if( virtual_base == MAP_FAILED ) {
        close( fd );
        throw std::runtime_error(std::string("mmap() failed"));
    }

//-----motor addresses----------------------
    h2p_lw_heartbeat_addr = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + HEARTBEAT_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_pwm_values_addr[0] = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_pwm_values_addr[1] = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_1_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_pwm_values_addr[2] = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_2_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_pwm_values_addr[3] = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_3_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_pwm_values_addr[4] = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_4_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_pwm_values_addr[5] = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_5_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_pwm_values_addr[6] = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_6_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_pwm_values_addr[7] = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_7_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_gpio1_addr=static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + GPIO_PIO_1_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    for(int k=0;k<8;k++){
        alt_write_word(h2p_lw_pwm_values_addr[k], 0);
    }
//------------------------------------------
}

void fpgaEncoderAddressesSetup(){
    //---fpga encoder addresses-----------------
    h2p_lw_quad_addr[0]=static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_quad_addr[1]=static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_1_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_quad_addr[2]=static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_2_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_quad_addr[3]=static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_3_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_quad_addr[4]=static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_4_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_quad_addr[5]=static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_5_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_quad_addr[6]=static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_6_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_quad_addr[7]=static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_7_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_lw_quad_reset_addr = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_RESET_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
//------------------------------------------
}



void miscAddressesSetup(){
    uint32_t reset_mask = 255 | 255<<20;

    h2p_lw_adc = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ADC_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

    alt_write_word(h2p_lw_adc, 0);


    alt_write_word(h2p_lw_quad_reset_addr, 0);

    h2p_lw_gpio0_addr = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + GPIO_PIO_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

    alt_write_word(h2p_lw_gpio0_addr, (1<<0) | (1<<10));
}


//reload all values of config files.
void loadINIConfigr(){
    INIReader reader("../robotConfig.ini");
    if (reader.ParseError() < 0) {
        throw std::runtime_error(std::string("cannot load config file."));
    }
//    PID
    P_float         = reader.GetReal("PID","P_float", -1);
    I_float         = reader.GetReal("PID","I_float", -1);
    D_float         = reader.GetReal("PID","D_float", -1);
    sampleRate    = reader.GetInteger("PID","SAMPLE_RATE", -1);
//---------------------------------

//safety limits
    PWMCap                         = reader.GetInteger("SAFETY","PWMCap", 0);
    InsertionJointLimitUpper = reader.GetInteger("SAFETY","InsertionJointLimitUpper", -9999);
    InsertionJointLimitLower = reader.GetInteger("SAFETY","InsertionJointLimitLower", -9999);
    TipJointLimitUpper          = reader.GetInteger("SAFETY","TipJointLimitUpper", -9999);
    TipJointLimitLower          = reader.GetInteger("SAFETY","TipJointLimitLower", -9999);
    MidJointLimitUpper         = reader.GetInteger("SAFETY","MidJointLimitUpper", -9999);
    MidJointLimitLower          = reader.GetInteger("SAFETY","MidJointLimitLower", -9999);
    BackJointLimitUpper        = reader.GetInteger("SAFETY","BackJointLimitUpper", -9999);
    BackJointLimitLower        = reader.GetInteger("SAFETY","BackJointLimitLower", -9999);
    RotationJointLimitUpper   = reader.GetInteger("SAFETY","RotationJointLimitUpper", -9999);
    RotationJointLimitLower   = reader.GetInteger("SAFETY","RotationJointLimitLower", -9999);

    LinearAxisUpperLimit        = reader.GetInteger("SAFETY","LinearAxisUpperLimit", -9999);
    LinearAxisLowerLimit        = reader.GetInteger("SAFETY","LinearAxisLowerLimit", -9999);
    VerticalAxisUpperLimit      = reader.GetInteger("SAFETY","VerticalAxisUpperLimit", -9999);
    VerticalAxisLowerLimit      = reader.GetInteger("SAFETY","VerticalAxisLowerLimit", -9999);
    HorizontalAxisUpperLimit  = reader.GetInteger("SAFETY","HorizontalAxisUpperLimit", -9999);
    HorizontalAxisLowerLimit  = reader.GetInteger("SAFETY","HorizontalAxisLowerLimit", -9999);

    InsertionMotorCurrentLimit        = reader.GetInteger("SAFETY","InsertionMotorCurrentLimit", -9999);
    TipMotorCurrentLimit                 = reader.GetInteger("SAFETY","TipMotorCurrentLimit", -9999);
    MidMotorCurrentLimit                = reader.GetInteger("SAFETY","HorizontalAxisLowerLimit", -9999);
    BackMotorCurrentLimit               = reader.GetInteger("SAFETY","HorizontalAxisLowerLimit", -9999);
    RotationMotorCurrentLimit          = reader.GetInteger("SAFETY","HorizontalAxisLowerLimit", -9999);
    LinearAxisMotorCurrentLimit       = reader.GetInteger("SAFETY","HorizontalAxisLowerLimit", -9999);
    VerticalAxisMotorCurrentLimit     = reader.GetInteger("SAFETY","HorizontalAxisLowerLimit", -9999);
    HorizontalAxisMotorCurrentLimit = reader.GetInteger("SAFETY","HorizontalAxisLowerLimit", -9999);

    motorBackOffset                        = reader.GetInteger("GENERAL","motorBackOffset", 0);
    motorTipOffset                           = reader.GetInteger("GENERAL","motorTipOffset", 0);
    motorMidOffset                          = reader.GetInteger("GENERAL","motorMidOffset", 0);


}


//input  thread
void readInputFunc(){
    std::string theUserInput;
    std::stringstream ss;
    std::cout<<'>';
    do {
        if(std::getline(std::cin, theUserInput)) {
            mtx.lock();
            if(theUserInput.length()) {
                if(theUserInput == "quit") {
                    exit_flag = 1;
                    exit_flag_main = 1;
                }

                else if(theUserInput == "reload") {
                    loadINIConfigr();
                    motorStack[7].setPIDValue(P_float, I_float, D_float, 1.0/sampleRate);
                    motorStack[5].setPIDValue(P_float, I_float, D_float, 1.0/sampleRate);
                    motorStack[4].setPIDValue(P_float, I_float, D_float, 1.0/sampleRate);
                    motorStack[6].setPIDValue(P_float, I_float, D_float, 1.0/sampleRate);
                }

                else if(theUserInput == "printPID")
                    std::cout << "PID Values"<< std::endl << "P: " << P_float << " I: " << I_float << " D: " << D_float<<std::endl;

                else if(theUserInput == "showmotorpos")
                    std::cout <<setPosMInsertion<<" "<<setPosMTip<<" "<<setPosMMid<<" "<<setPosMBack<<std::endl;

                else{
                    ss = std::stringstream(theUserInput);
                    while(ss) {
                        ss >> setPosMInsertion >> setPosMTip  >> setPosMMid >> setPosMBack;
                    }
                    setPosMInsertion = (setPosMInsertion > TipJointLimitUpper)? TipJointLimitUpper : setPosMInsertion;
                    setPosMInsertion = (setPosMInsertion < TipJointLimitLower)?  TipJointLimitLower : setPosMInsertion;

                    setPosMTip = (setPosMTip > TipJointLimitUpper)? TipJointLimitUpper : setPosMTip;
                    setPosMTip = (setPosMTip < TipJointLimitLower)?  TipJointLimitLower : setPosMTip;

                    setPosMBack = (setPosMBack > TipJointLimitUpper)? TipJointLimitUpper : setPosMBack;
                    setPosMBack = (setPosMBack < TipJointLimitLower)?  TipJointLimitLower : setPosMBack;

                    setPosMMid = (setPosMMid > TipJointLimitUpper)? TipJointLimitUpper : setPosMMid;
                    setPosMMid = (setPosMMid < TipJointLimitLower)?  TipJointLimitLower : setPosMMid;
                }
            }
            mtx.unlock();
            usleep(10);
        }
//        motorMid.readI2CEncoder();
    }
    while (!exit_flag);
};




#endif //TRYING_MAIN_H
