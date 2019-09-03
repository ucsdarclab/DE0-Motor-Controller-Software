//
// Created by bunny on 8/19/19.
//

#ifndef MOTOR_TEST_1_MOTORCONTROLLER_H
#define MOTOR_TEST_1_MOTORCONTROLLER_H

extern "C"{
#include <rc/math/filter.h>
}

#include <map>
#include <cmath>
#include "fpgaAddressComm.h"
#include "fpgaEncoder.h"
#include "AS5048b.h"

namespace CTRobot{

    class motorController{

    public:
        motorController(unsigned char* PWMAddress, unsigned char* dirAddress, unsigned char* ADCAddr, uint32_t anIndex):
                motorPWM(PWMAddress), motorDirection(dirAddress),motorADC(ADCAddr),
                motorIndex(anIndex), motorEnable(NULL), motorEncoder(nullptr), i2cEncoder(nullptr){

            PIDfilter = rc_filter_empty();
            motorPWM.writeWord(0);
        };

//        motorController(const motorController &aController){
//            motorPWM = aController.motorPWM;
//            motorDirection = aController.motorDirection;
//            motorADC = aController.motorADC;
//            motorIndex = aController.motorIndex;
//            motorEnable = aController.motorEnable;
//            motorEncoder = aController.motorEncoder;
//            i2cEncoder = aController.i2cEncoder;
//        }

        ~motorController(){
            if(!motorEncoder)
                delete(motorEncoder);
            if(!i2cEncoder)
                delete(i2cEncoder);

        }
//        when speed is 0 move function would be a stop function aswell, direction would not matert
        bool move(uint32_t aSpeed, keywords aKeyword);
        void stop();
//        attach fpga encoder
        motorController& attachEncoder(unsigned char* aEncoderAddr, unsigned char* encoderResetAddr, uint32_t anIndex);
//        attach i2c encoder
        motorController& attachEncoder(I2CBus* aBus, uint8_t anI2cAddr);

        int32_t readEncoder();
        double readI2CEncoder(keywords aKeywords = keywords::degree);
        void resetEncoder(keywords aKeyword = keywords::FPGA);
//        get motor index
        uint32_t getIndex();

        int32_t readADC();

//------PID function for individual motor-----
        motorController& setPIDValue(float aP = 0.0008, float aI = 0.0002, float aD = 0.000005, float dt = 0.001);
        void runPID(int32_t aDestination, keywords aKeyword);
//--------------------------------------------

    protected:
        fpgaCommunication motorPWM;
        fpgaCommunication motorDirection;
        fpgaCommunication motorEnable;
        fpgaCommunication motorADC;
        fpgaEncoder *motorEncoder;
        AS5048b *i2cEncoder;
        uint32_t motorIndex;
        rc_filter_t PIDfilter;
    };
}

#endif //MOTOR_TEST_1_MOTORCONTROLLER_H
