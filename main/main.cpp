#include "main.h"


using Eigen::MatrixXd;

/*------------------------------------------------------------------------------------------------
 * main
 *------------------------------------------------------------------------------------------------*/
int main(int args, char** argc){
//    initialization
    signal(SIGINT, signalHandler);

    motorAddressesSetup();

    fpgaEncoderAddressesSetup();

    miscAddressesSetup();

    loadINIConfigr();
//    end of initialization


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


//end of heartbeat thread

//setup motors and its encoders


    auto motorFunc = [](){

        long long loopCounter = 0;
        double usAverageLoopTime = 0;

//      initialization for for all motors objects
        motorBack = CTRobot::motorController (h2p_lw_pwm_values_addr[0], h2p_lw_gpio1_addr, h2p_lw_adc, 7);
        motorInsertion = CTRobot::motorController (h2p_lw_pwm_values_addr[1],h2p_lw_gpio1_addr, h2p_lw_adc, 6);
        motorTip = CTRobot::motorController (h2p_lw_pwm_values_addr[2], h2p_lw_gpio1_addr, h2p_lw_adc, 5);
        motorMid = CTRobot::motorController (h2p_lw_pwm_values_addr[3], h2p_lw_gpio1_addr, h2p_lw_adc, 4);
        motorRotation = CTRobot::motorController (h2p_lw_pwm_values_addr[4], h2p_lw_gpio1_addr, h2p_lw_adc, 3);
        motorLinear = CTRobot::motorController (h2p_lw_pwm_values_addr[5],h2p_lw_gpio1_addr, h2p_lw_adc, 2);
        motorVertical = CTRobot::motorController (h2p_lw_pwm_values_addr[6], h2p_lw_gpio1_addr, h2p_lw_adc, 1);
        motorHorizontal = CTRobot::motorController (h2p_lw_pwm_values_addr[7], h2p_lw_gpio1_addr, h2p_lw_adc, 0);


        motorBack.attachEncoder(h2p_lw_quad_addr[0], h2p_lw_quad_reset_addr, 7);
        motorBack.attachEncoder(theBus, 0x44, motorBackOffset);
//        motorBack.resetEncoder(CTRobot::keywords::FPGA);
//        motorBack.resetEncoder(CTRobot::keywords::I2C);

        motorInsertion.attachEncoder(h2p_lw_quad_addr[1], h2p_lw_quad_reset_addr, 6);


        motorTip.attachEncoder(h2p_lw_quad_addr[2], h2p_lw_quad_reset_addr, 5);
        motorTip.attachEncoder(theBus, 0x41, motorTipOffset);

//        motorTip.resetEncoder(CTRobot::keywords::I2C);

        motorMid.attachEncoder(h2p_lw_quad_addr[3], h2p_lw_quad_reset_addr, 4);
        motorMid.attachEncoder(theBus, 0x40, motorMidOffset);

//        motorMid.resetEncoder(CTRobot::keywords::I2C);

        motorRotation.attachEncoder(h2p_lw_quad_addr[4], h2p_lw_quad_reset_addr, 3);
        motorRotation.resetEncoder(CTRobot::keywords::FPGA);

        motorLinear.attachEncoder(h2p_lw_quad_addr[5], h2p_lw_quad_reset_addr, 2);
//        motorLinear.resetEncoder(CTRobot::keywords::FPGA);

        motorVertical.attachEncoder(h2p_lw_quad_addr[6], h2p_lw_quad_reset_addr, 1);
//        motorVertical.resetEncoder(CTRobot::keywords::FPGA);

        motorHorizontal.attachEncoder(h2p_lw_quad_addr[7], h2p_lw_quad_reset_addr, 0);
//        motorHorizontal.resetEncoder(CTRobot::keywords::FPGA);
//        homeMotor();



        auto systemStart = std::chrono::high_resolution_clock::now();

        //motorTip.setPIDValue(0.005, 0.00002, 0.0001, 0.001);

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
//        std::cout << pulleyToMotorRatio << std::endl;
//        std::cout << motorRadiansToCounts << std::endl;
//        std::cout << shaftAngleToCounts << std::endl;

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

//        std::cout << "setpoint: " << setpoint << std::endl;
//        std::cout << "mixing matrix 1: " << mixingMatrix1 << std::endl << "mixing matrix 2: " << mixingMatrix2 << std::endl << "mixing matrix 3: " << mixingMatrix3 << std::endl << "mixing matrix 4: " << mixingMatrix4 << std::endl;
//        std::cout << "mixing matrix: " << fullMixingMatrix << std::endl << "inverse mixing matrix: " << finalMixingMatrix << std::endl;
//        std::cout << "output: " << finalMixingMatrix * setpoint << std::endl;
/*------------------------------------------------------------------------------------------------
* control loop
*------------------------------------------------------------------------------------------------*/
//        auto loopStart = clock();
        MatrixXd temp;

        auto loopStart = std::chrono::high_resolution_clock::now();
        auto loopEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::micro> duration = loopStart - loopEnd;
        std::chrono::microseconds dt_chrono((int)(std::pow(10,6)*dt));

        double angleTemp = 180;
        motorTip.setPIDValue(0.05, I_float, 0.00001, 10.0/sampleRate);


        while(exit_flag == 0 and fabs(angleTemp) >= 1){
            motorTip.runPID(0,CTRobot::keywords::I2C);
            angleTemp = motorTip.readI2CEncoder();
            std::cout << angleTemp <<std::endl;
            usleep(10000);
        }


        motorInsertion.resetEncoder(CTRobot::keywords::FPGA);
        motorTip.resetEncoder(CTRobot::keywords::FPGA);
        motorMid.resetEncoder(CTRobot::keywords::FPGA);
        motorBack.resetEncoder(CTRobot::keywords::FPGA);

        motorBack.setPIDValue(P_float, I_float, D_float, 1.0/sampleRate);
        motorInsertion.setPIDValue(P_float, I_float, D_float, 1.0/sampleRate);
        motorTip.setPIDValue(P_float, I_float, D_float, 1.0/sampleRate);
        motorMid.setPIDValue(P_float, I_float, D_float, 1.0/sampleRate);

        while(exit_flag == 0) {
            //duration = (double) (clock() - start) / CLOCKS_PER_SEC * 1000000;

            loopStart = std::chrono::high_resolution_clock::now();

            std::chrono::duration<double> timeSinceStart = loopStart- systemStart;
//            int32_t setpointTest = (int32_t)(sin(timeSinceStart * 2 * M_PI / 1) * M_PI/8);
//            setpoint(0) = sin(timeSinceStart.count() * 2 * M_PI * 0.05) * M_PI/3;
            setpoint(0) = (float)setPosMBack/360.0 * 2 * M_PI;
            setpoint(1) = (float)setPosMMid/360.0 * 2 * M_PI;
            setpoint(2) = (float)setPosMTip/360.0 * 2 * M_PI;
            setpoint(3) = (float)setPosMInsertion/360.0 * 2 * M_PI;
            //start = clock();

//		we only need to calculate the matrix once every time!!!!!!!!
            temp = shaftAngleToCounts * finalMixingMatrix * setpoint;


//                motorTip.runPID(100,CTRobot::keywords::FPGA);
//            std::cout<<motorTip.readI2CEncoder(CTRobot::keywords::degree) << "    "<<
//            motorMid.readI2CEncoder(CTRobot::keywords::degree)<<"    "<<
//            motorBack.readI2CEncoder(CTRobot::keywords::degree)<<std::endl;
//                motorTip.runPID(setPosMTip,CTRobot::keywords::I2C);
            motorBack.runPID((int32_t)temp(0),CTRobot::keywords::FPGA);
            motorInsertion.runPID((int32_t)temp(3),CTRobot::keywords::FPGA);
            motorTip.runPID((int32_t)temp(2),CTRobot::keywords::FPGA);
            motorMid.runPID((int32_t)temp(1),CTRobot::keywords::FPGA);


//            motorTip.move(150, CTRobot::keywords::forward);


            loopEnd = std::chrono::high_resolution_clock::now();
            duration = loopEnd - loopStart;

            usAverageLoopTime = usAverageLoopTime * (float)loopCounter / (float)(loopCounter + 1)
                                + (float)std::chrono::duration_cast<std::chrono::microseconds>(duration).count()/(float)(loopCounter+1);
            loopCounter++;
            if(duration >= dt_chrono*1.0)
                std::cout<<"overun: "<< std::chrono::duration_cast<std::chrono::microseconds>(duration).count() << " us " << "average loop time: " << usAverageLoopTime << " us" << std::endl;
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
    std::thread inputThread(readInputFunc);
    std::thread motorThread(motorFunc);


    heartBeatThread.join();
    usleep(1000);
    motorThread.join();
    inputThread.join();




    delete(theBus);
    return 0;
}