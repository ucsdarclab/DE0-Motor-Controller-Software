#include "main.h"
//#include "../include/DE0_Motor_Controller/fpgaAddressComm.h"



/*------------------------------------------------------------------------------------------------
 * main
 *------------------------------------------------------------------------------------------------*/
int main(int args, char** argc){
//    initialization

    motorAddressesSetup();

    fpgaEncoderAddressesSetup();

    miscAddressesSetup();

    loadINIConfigr();
//    end of initialization


//heartbeat thread


//end of heartbeat thread

//setup motors and its encoders


motorStack.emplace_back(CTRobot::motorController (h2p_lw_pwm_values_addr[0], h2p_lw_gpio1_addr, h2p_lw_adc, 0));
motorStack.emplace_back(CTRobot::motorController (h2p_lw_pwm_values_addr[1],h2p_lw_gpio1_addr, h2p_lw_adc, 1));
motorStack.emplace_back(CTRobot::motorController (h2p_lw_pwm_values_addr[2], h2p_lw_gpio1_addr, h2p_lw_adc, 2));
motorStack.emplace_back(CTRobot::motorController (h2p_lw_pwm_values_addr[3], h2p_lw_gpio1_addr, h2p_lw_adc, 3));
motorStack.emplace_back(CTRobot::motorController (h2p_lw_pwm_values_addr[4], h2p_lw_gpio1_addr, h2p_lw_adc, 4));
motorStack.emplace_back(CTRobot::motorController (h2p_lw_pwm_values_addr[5],h2p_lw_gpio1_addr, h2p_lw_adc, 5));
motorStack.emplace_back(CTRobot::motorController (h2p_lw_pwm_values_addr[6], h2p_lw_gpio1_addr, h2p_lw_adc, 6));
motorStack.emplace_back(CTRobot::motorController (h2p_lw_pwm_values_addr[7], h2p_lw_gpio1_addr, h2p_lw_adc, 7));

motorStack[0].attachEncoder(h2p_lw_quad_addr[0], h2p_lw_quad_reset_addr, 0);
motorStack[1].attachEncoder(h2p_lw_quad_addr[1], h2p_lw_quad_reset_addr, 1);
motorStack[2].attachEncoder(h2p_lw_quad_addr[2], h2p_lw_quad_reset_addr, 2);
motorStack[3].attachEncoder(h2p_lw_quad_addr[3], h2p_lw_quad_reset_addr, 3);
motorStack[4].attachEncoder(h2p_lw_quad_addr[4], h2p_lw_quad_reset_addr, 4);
motorStack[5].attachEncoder(h2p_lw_quad_addr[5], h2p_lw_quad_reset_addr, 5);
motorStack[6].attachEncoder(h2p_lw_quad_addr[6], h2p_lw_quad_reset_addr, 6);
motorStack[7].attachEncoder(h2p_lw_quad_addr[7], h2p_lw_quad_reset_addr, 7);

motorStack[3].setPIDValue(0.001, 0.0000, 0.0000, 1.0/1000.0);
motorStack[3].resetEncoder(CTRobot::keywords::FPGA);
auto systemStart = std::chrono::high_resolution_clock::now();
auto loopStart = std::chrono::high_resolution_clock::now();
auto loopEnd = std::chrono::high_resolution_clock::now();
std::chrono::duration<double, std::micro> duration = loopStart - loopEnd;
std::chrono::microseconds dt_chrono((int)(std::pow(10,6)*0.001));
double usAverageLoopTime = 0;
long long loopCounter = 0;

while(true){
//    motorStack[3].move(1000,CTRobot::keywords::backward);
    loopStart = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> timeSinceStart = loopStart - systemStart;
    double setpointTest = (double)(sin(timeSinceStart.count() * 2 * M_PI / 5) * M_PI/3);
//    std::cout<<setpointTest<<std::endl;
    motorStack[3].runPID(200000*setpointTest,CTRobot::keywords::FPGA);
//    motorStack[3].move(500,CTRobot::keywords::forward);
    std::cout<<motorStack[3].readEncoder()<<" "<<200000*setpointTest<<std::endl;

//    usleep(990);
////    std::cout<<"running"<<std::endl;
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



    delete(theBus);
    return 0;
}