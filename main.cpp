#include "main.h"
#include <ctime>
#include <fstream>
#include <Eigen/Dense>
#include <cmath>

#include <iostream>
#include <chrono>
#include <thread>


/*------------------------------------------------------------------------------------------------
 * setup
 *------------------------------------------------------------------------------------------------*/
int32_t setPosMBack(0); int32_t setPosMInsertion(0); int32_t setPosMTip(0); int32_t setPosMMid(0);

unsigned char* h2p_lw_gpio0_addr;
unsigned char* h2p_lw_heartbeat_addr;
unsigned char* h2p_lw_quad_reset_addr;
unsigned char* h2p_lw_quad_addr[8];
unsigned char* h2p_lw_pwm_values_addr[8];
unsigned char* h2p_lw_adc;
unsigned char* h2p_lw_gpio1_addr;

std::mutex mtx;

int CURRENT_FLAG;
int TRAVEL_FLAG;
int ETSOP_FLAG;
int exit_flag;
double dt = 0.001;

int fd;
void* virtual_base;


void signalHandler(int sigNum){
    exit_flag = 1;
    std::cout << "setting exit_flag to 1" << std:: endl;
}

//-----i2c sensor bus--------------------------
std::string busName = "/dev/i2c-1";
auto *theBus = new CTRobot::I2CBus(busName);
//---------------------------------------------

using Eigen::MatrixXd;

/*------------------------------------------------------------------------------------------------
 * main
 *------------------------------------------------------------------------------------------------*/
int main(int args, char** argc){
    signal(SIGINT, signalHandler);


//initialization
    exit_flag = 0;

    if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
        printf( "ERROR: could not open \"/dev/mem\"...\n" );
        return( 1 );
    }
    virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );
    if( virtual_base == MAP_FAILED ) {
        printf( "ERROR: mmap() failed...\n" );
        close( fd );
        return(1);
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

    uint32_t reset_mask = 255 | 255<<20;

    h2p_lw_adc = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ADC_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

//    alt_write_word(h2p_lw_adc, 0);


    alt_write_word(h2p_lw_quad_reset_addr, 0);

    h2p_lw_gpio0_addr = static_cast<unsigned char*>(virtual_base) + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + GPIO_PIO_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

    alt_write_word(h2p_lw_gpio0_addr, (1<<0) | (1<<10));
//end of initialization


//heartbeat thread
    auto heartBeatFunc = [](){
        CTRobot::fpgaHeartBeat heartBeat(h2p_lw_heartbeat_addr);
        std::cout<< "heartbeat on"<<std::endl;

        while (exit_flag == 0) {
            heartBeat.changeState();
            usleep(10000);
        }
        std::cout<< "heartbeat shutting down"<<std::endl;
    };
//input  thread
    auto readInputFunc = [](){
        std::string theUserInput;
        do {
            std::cout << std::endl << "> ";
            if(std::getline(std::cin, theUserInput)) {
                mtx.lock();
                if(theUserInput.length()) {
                    if(theUserInput == "quit") exit_flag = 1;
                    else{
                        setPosMTip = atoi(theUserInput.c_str());
                    }
                }
                mtx.unlock();
                usleep(10);
            }
        }
        while (!exit_flag);
    };


//end of heartbeat thread

//setup motors and its encoders


    auto motorFunc = [](){

        long long loopCounter = 0;
        double usAverageLoopTime = 0;

//      initialization for for all motors objects
        CTRobot::motorController motorBack(h2p_lw_pwm_values_addr[0], h2p_lw_gpio1_addr, h2p_lw_adc, 7);
        CTRobot::motorController motorInsertion(h2p_lw_pwm_values_addr[1],h2p_lw_gpio1_addr, h2p_lw_adc, 6);
        CTRobot::motorController motorTip(h2p_lw_pwm_values_addr[2], h2p_lw_gpio1_addr, h2p_lw_adc, 5);
        CTRobot::motorController motorMid(h2p_lw_pwm_values_addr[3], h2p_lw_gpio1_addr, h2p_lw_adc, 4);
        CTRobot::motorController motorRotation(h2p_lw_pwm_values_addr[4], h2p_lw_gpio1_addr, h2p_lw_adc, 3);
        CTRobot::motorController motorLinear(h2p_lw_pwm_values_addr[5],h2p_lw_gpio1_addr, h2p_lw_adc, 2);
        CTRobot::motorController motorVertical(h2p_lw_pwm_values_addr[6], h2p_lw_gpio1_addr, h2p_lw_adc, 1);
        CTRobot::motorController motorHorizontal(h2p_lw_pwm_values_addr[7], h2p_lw_gpio1_addr, h2p_lw_adc, 0);

        motorBack.attachEncoder(h2p_lw_quad_addr[0], h2p_lw_quad_reset_addr, 7);
        motorBack.attachEncoder(theBus, 0x44);
        motorBack.resetEncoder(CTRobot::keywords::FPGA);
        motorBack.resetEncoder(CTRobot::keywords::I2C);

        motorInsertion.attachEncoder(h2p_lw_quad_addr[1], h2p_lw_quad_reset_addr, 6);
        motorInsertion.resetEncoder(CTRobot::keywords::FPGA);

        motorTip.attachEncoder(h2p_lw_quad_addr[2], h2p_lw_quad_reset_addr, 5);
        motorTip.attachEncoder(theBus, 0x4d);
        motorTip.resetEncoder(CTRobot::keywords::FPGA);
        motorTip.resetEncoder(CTRobot::keywords::I2C);

        motorMid.attachEncoder(h2p_lw_quad_addr[3], h2p_lw_quad_reset_addr, 4);
        motorMid.attachEncoder(theBus, 0x40);
        motorMid.resetEncoder(CTRobot::keywords::FPGA);
        motorMid.resetEncoder(CTRobot::keywords::I2C);

        motorRotation.attachEncoder(h2p_lw_quad_addr[4], h2p_lw_quad_reset_addr, 3);
        motorRotation.resetEncoder(CTRobot::keywords::FPGA);

        motorLinear.attachEncoder(h2p_lw_quad_addr[5], h2p_lw_quad_reset_addr, 2);
        motorLinear.resetEncoder(CTRobot::keywords::FPGA);

        motorVertical.attachEncoder(h2p_lw_quad_addr[6], h2p_lw_quad_reset_addr, 1);
        motorVertical.resetEncoder(CTRobot::keywords::FPGA);

        motorHorizontal.attachEncoder(h2p_lw_quad_addr[7], h2p_lw_quad_reset_addr, 0);
        motorHorizontal.resetEncoder(CTRobot::keywords::FPGA);

        auto systemStart = std::chrono::high_resolution_clock::now();

        double duration;

        //motorTip.setPIDValue(0.005, 0.00002, 0.0001, 0.001);
        motorBack.setPIDValue(0.00005, 0.0000, 0.0000002, dt);
        motorInsertion.setPIDValue(0.00005, 0.0000, 0.0000002, dt);
        motorTip.setPIDValue(0.00005, 0.0000, 0.0000002, dt);
        motorMid.setPIDValue(0.00005, 0.0000, 0.0000002, dt);

        uint32_t ADCVal = 0;
        uint32_t prevADCVal = 0;
//        motorTip.move(150,CTRobot::keywords::backward);
//        usleep(1000);
//        while(exit_flag == 0 ){
//            ADCVal = abs(motorTip.readADC() - 1651) * 0.2 + prevADCVal * 0.8;
//            prevADCVal = ADCVal;
//            std::cout<<ADCVal<<std::endl;
//        }
//        motorTip.stop();
//        motorTip.resetEncoder(CTRobot::keywords::I2C);


        MatrixXd motorRadiansToCounts(4,4);
        motorRadiansToCounts(0,0) = 44 * 2048 / (2 * M_PI);
        motorRadiansToCounts(1,1) = 44 * 2048 / (2 * M_PI);
        motorRadiansToCounts(2,2) = 44 * 2048 / (2 * M_PI);
        motorRadiansToCounts(3,3) = 44 * 2048 / (2 * M_PI);

        MatrixXd pulleyToMotorRatio (4,4);
        pulleyToMotorRatio(0,0) = 50.0/22.0;
        pulleyToMotorRatio(1,1) = 50.0/22.0;
        pulleyToMotorRatio(2,2) = 50.0/22.0;
        pulleyToMotorRatio(3,3) = 50.0/22.0;

        MatrixXd shaftAngleToCounts = pulleyToMotorRatio * motorRadiansToCounts;
        std::cout << pulleyToMotorRatio << std::endl;
        std::cout << motorRadiansToCounts << std::endl;
        std::cout << shaftAngleToCounts << std::endl;

        MatrixXd mixingMatrix1 = MatrixXd::Identity(4,4);
        MatrixXd mixingMatrix2 = MatrixXd::Identity(4,4);
        MatrixXd mixingMatrix3 = MatrixXd::Identity(4,4);
        MatrixXd mixingMatrix4 = MatrixXd::Identity(4,4);
//        matrix 1
        mixingMatrix1(0,0) = 13.7 / 28.0;

//        matrix 2
        mixingMatrix2(1,0) = 22.0 / 28.0;
        mixingMatrix2(1,1) = 13.7 / 28.0;

//       matrix 3
        mixingMatrix3(2,0) = 1.0;
        mixingMatrix3(2,1) = 28.0/22.0;
        mixingMatrix3(2,2)  = -13.7/22.0;

//       matrix 4
        mixingMatrix4(3,0) = 22.0;
        mixingMatrix4(3,1) = 28.0;
        mixingMatrix4(3,2) = -28.0;
        mixingMatrix4(3,3) = 13.7;

        //outputs joint angle given input shaft angle
        MatrixXd fullMixingMatrix = mixingMatrix4 * mixingMatrix3 * mixingMatrix2 * mixingMatrix1;

        //input shaft angle required for output joint angles: calculate inverse of matrix
        MatrixXd inverseMixingMatrix = fullMixingMatrix.inverse();
        MatrixXd finalMixingMatrix = inverseMixingMatrix;

        Eigen::Vector4d setpoint(M_PI/4,0,0,0);

        std::cout << "setpoint: " << setpoint << std::endl;
        std::cout << "mixing matrix 1: " << mixingMatrix1 << std::endl << "mixing matrix 2: " << mixingMatrix2 << std::endl << "mixing matrix 3: " << mixingMatrix3 << std::endl << "mixing matrix 4: " << mixingMatrix4 << std::endl;
        std::cout << "mixing matrix: " << fullMixingMatrix << std::endl << "inverse mixing matrix: " << finalMixingMatrix << std::endl;
        std::cout << "output: " << finalMixingMatrix * setpoint << std::endl;
/*------------------------------------------------------------------------------------------------
* control loop
*------------------------------------------------------------------------------------------------*/
//        auto loopStart = clock();
        MatrixXd temp;
        while(exit_flag == 0) {
            //duration = (double) (clock() - start) / CLOCKS_PER_SEC * 1000000;

            auto loopStart = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> timeSinceStart = loopStart- systemStart;
            //int32_t setpointTest = (int32_t)(sin(timeSinceStart * 2 * M_PI / 1) * M_PI/8);
            setpoint(0) = sin(timeSinceStart.count() * 2 * M_PI * 0.05) * M_PI/3;

            //start = clock();

//		we only need to calculate the matrix once every time!!!!!!!!
            temp = shaftAngleToCounts * finalMixingMatrix * setpoint;


//                motorTip.runPID(100000 * std::sin(2 * 3.14159 * 0.25* (double) ((double) clock() / CLOCKS_PER_SEC)),
//                                CTRobot::keywords::FPGA);
//                motorTip.runPID(100,CTRobot::keywords::FPGA);
//            std::cout<<motorTip.readI2CEncoder(CTRobot::keywords::degree) << "    "<<
//            motorMid.readI2CEncoder(CTRobot::keywords::degree)<<"    "<<
//            motorBack.readI2CEncoder(CTRobot::keywords::degree)<<std::endl;
//                motorTip.runPID(setPosMTip,CTRobot::keywords::I2C);
            motorBack.runPID((int32_t)temp(0),CTRobot::keywords::FPGA);
            motorInsertion.runPID((int32_t)temp(3),CTRobot::keywords::FPGA);
            motorTip.runPID((int32_t)temp(2),CTRobot::keywords::FPGA);
            motorMid.runPID((int32_t)temp(1),CTRobot::keywords::FPGA);

            auto loopEnd = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::micro> duration = loopEnd - loopStart;
            std::chrono::microseconds dt_chrono((int)(std::pow(10,6)*dt));
            usAverageLoopTime = usAverageLoopTime * loopCounter / (loopCounter + 1)
                    + std::chrono::duration_cast<std::chrono::microseconds>(duration).count()/(loopCounter+1);
            loopCounter++;

            if(duration >= dt_chrono*1.0)
                int trash = 0;
                //std::cout<<"overun: "<< std::chrono::duration_cast<std::chrono::microseconds>(duration).count() << " us " << "average loop time: " << usAverageLoopTime << " us" << std::endl;
            else
                std::this_thread::sleep_for(dt_chrono - duration);

        }

        //loopStart = clock();

//        do{
//            motorBack.runPID(0, CTRobot::keywords::FPGA);
//            motorInsertion.runPID(0, CTRobot::keywords::FPGA);
//            motorTip.runPID(0, CTRobot::keywords::FPGA);
//            motorMid.runPID(0, CTRobot::keywords::FPGA);
//            usleep(1000);
//        }while((double) (clock() - loopStart) / CLOCKS_PER_SEC < 5);

        motorTip.stop();
        motorMid.stop();
        motorInsertion.stop();
        motorBack.stop();
    };

    std::thread heartBeatThread(heartBeatFunc);
    std::thread motorThread(motorFunc);
    std::thread inputThread(readInputFunc);

    heartBeatThread.join();
    usleep(1000);
    inputThread.join();
    motorThread.join();


    delete(theBus);
    return 0;
}