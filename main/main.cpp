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
        motorStack.emplace_back(CTRobot::motorController (h2p_lw_pwm_values_addr[0], h2p_lw_gpio1_addr, h2p_lw_adc, 7));
        motorStack.emplace_back(CTRobot::motorController (h2p_lw_pwm_values_addr[1],h2p_lw_gpio1_addr, h2p_lw_adc, 6));
        motorStack.emplace_back(CTRobot::motorController (h2p_lw_pwm_values_addr[2], h2p_lw_gpio1_addr, h2p_lw_adc, 5));
        motorStack.emplace_back(CTRobot::motorController (h2p_lw_pwm_values_addr[3], h2p_lw_gpio1_addr, h2p_lw_adc, 4));
        motorStack.emplace_back(CTRobot::motorController (h2p_lw_pwm_values_addr[4], h2p_lw_gpio1_addr, h2p_lw_adc, 3));
        motorStack.emplace_back(CTRobot::motorController (h2p_lw_pwm_values_addr[5],h2p_lw_gpio1_addr, h2p_lw_adc, 2));
        motorStack.emplace_back(CTRobot::motorController (h2p_lw_pwm_values_addr[6], h2p_lw_gpio1_addr, h2p_lw_adc, 1));
        motorStack.emplace_back(CTRobot::motorController (h2p_lw_pwm_values_addr[7], h2p_lw_gpio1_addr, h2p_lw_adc, 0));

        motorStack[0].attachEncoder(h2p_lw_quad_addr[0], h2p_lw_quad_reset_addr, 7);
        motorStack[1].attachEncoder(h2p_lw_quad_addr[1], h2p_lw_quad_reset_addr, 6);
        motorStack[2].attachEncoder(h2p_lw_quad_addr[2], h2p_lw_quad_reset_addr, 5);
        motorStack[3].attachEncoder(h2p_lw_quad_addr[3], h2p_lw_quad_reset_addr, 4);
        motorStack[4].attachEncoder(h2p_lw_quad_addr[4], h2p_lw_quad_reset_addr, 3);
        motorStack[5].attachEncoder(h2p_lw_quad_addr[5], h2p_lw_quad_reset_addr, 2);
        motorStack[6].attachEncoder(h2p_lw_quad_addr[6], h2p_lw_quad_reset_addr, 1);
        motorStack[7].attachEncoder(h2p_lw_quad_addr[7], h2p_lw_quad_reset_addr, 0);

        motorStack[5].attachEncoder(theBus, 0x41, motorTipOffset);
        motorStack[6].attachEncoder(theBus, 0x40, motorMidOffset);
        motorStack[7].attachEncoder(theBus, 0x44, motorBackOffset);



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
        motorStack[5].setPIDValue(0.05, I_float, 0.00001, 10.0/sampleRate);


        motorStack[4].resetEncoder(CTRobot::keywords::FPGA);
        motorStack[5].resetEncoder(CTRobot::keywords::FPGA);
        motorStack[6].resetEncoder(CTRobot::keywords::FPGA);
        motorStack[7].resetEncoder(CTRobot::keywords::FPGA);

        motorStack[4].setPIDValue(P_float, I_float, D_float, 1.0/sampleRate);
        motorStack[5].setPIDValue(P_float, I_float, D_float, 1.0/sampleRate);
        motorStack[6].setPIDValue(P_float, I_float, D_float, 1.0/sampleRate);
        motorStack[7].setPIDValue(P_float, I_float, D_float, 1.0/sampleRate);

        while(exit_flag == 0) {
            //duration = (double) (clock() - start) / CLOCKS_PER_SEC * 1000000;

            loopStart = std::chrono::high_resolution_clock::now();

            std::chrono::duration<double> timeSinceStart = loopStart- systemStart;
//            int32_t setpointTest = (int32_t)(sin(timeSinceStart * 2 * M_PI / 1) * M_PI/8);
//            setpoint(0) = sin(timeSinceStart.count() * 2 * M_PI * 0.05) * M_PI/3;
            setpoint(0) = (float)0/360.0 * 2 * M_PI;
            setpoint(1) = (float)0/360.0 * 2 * M_PI;
            setpoint(2) = (float)setPosMTip/360.0 * 2 * M_PI;
            setpoint(3) = (float)0/360.0 * 2 * M_PI;
            //start = clock();

//		we only need to calculate the matrix once every time!!!!!!!!
            temp = shaftAngleToCounts * finalMixingMatrix * setpoint;

//            motorStack[4].runPID((int32_t)temp(3),CTRobot::keywords::FPGA, PWMCap);
            motorStack[5].runPID((int32_t)temp(2),CTRobot::keywords::FPGA, PWMCap);
//            motorStack[6].runPID((int32_t)temp(1),CTRobot::keywords::FPGA, PWMCap);
//            motorStack[7].runPID((int32_t)temp(0),CTRobot::keywords::FPGA, PWMCap);


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

        for(int i = 0; i <= 7; i+=1)
            motorStack[i].stop();
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