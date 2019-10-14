#include "main.h"
#include <pthread.h>
#include <sched.h>

using Eigen::MatrixXd;

void set_realtime_priority() {

    std::cout<<"inside"<<std::endl;

    int ret;
    // We'll operate on the currently running thread.
    pthread_t this_thread = pthread_self();

    // struct sched_param is used to store the scheduling priority
    struct sched_param params;

    // We'll set the priority to the maximum.
    params.sched_priority = sched_get_priority_max(SCHED_FIFO);

    std::cout << "Trying to set thread realtime prio = " << params.sched_priority << std::endl;

    // Attempt to set thread real-time priority to the SCHED_FIFO policy
    ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);
    if (ret != 0) {
        // Print the error
        std::cout << "Unsuccessful in setting thread realtime prio" << std::endl;
        return;
    }
    // Now verify the change in thread priority
    int policy = 0;
    ret = pthread_getschedparam(this_thread, &policy, &params);
    if (ret != 0) {
        std::cout << "Couldn't retrieve real-time scheduling paramers" << std::endl;
        return;
    }

    // Check the correct policy was applied
    if(policy != SCHED_FIFO) {
        std::cout << "Scheduling is NOT SCHED_FIFO!" << std::endl;
    } else {
        std::cout << "SCHED_FIFO OK" << std::endl;
    }

    // Print thread scheduling priority
    std::cout << "Thread priority is " << params.sched_priority << std::endl;
}
/*------------------------------------------------------------------------------------------------
 * main
 *------------------------------------------------------------------------------------------------*/
int main(int argc, char** argv){
    signal(SIGINT, signalHandler);

    //Load config file
    if (argc > 1) {
        loadINIConfigr(argv[1]);
    } else {
        std::cerr << "Error: Load config file" <<  std::endl;
        std::cerr << "Usage: " << argv[0] << " [config]" <<  std::endl;
        std::cerr << "    config - the path to the config file" <<  std::endl;
        return -1;
    }

    //initialization
    motorAddressesSetup();
    fpgaEncoderAddressesSetup();
    miscAddressesSetup();
    //end of initialization

    //heartbeat thread
    auto heartBeatFunc = [](){
        CTRobot::fpgaHeartBeat heartBeat(h2p_lw_heartbeat_addr);
        std::cout<< "heartbeat on"<<std::endl;

        while (exit_flag == 0 || zeroed == 0) {
            heartBeat.changeState();
            usleep(10000);
        }
        std::cout<< "heartbeat shutting down"<<std::endl;
    }; //end of heartbeat thread
    std::thread heartBeatThread(heartBeatFunc);


    //initialize motors
    motorBack = CTRobot::motorController (h2p_lw_pwm_values_addr[0], h2p_lw_gpio1_addr, h2p_lw_adc, 7); //back motor/board
    motorBack.attachEncoder(h2p_lw_quad_addr[0], h2p_lw_quad_reset_addr, 7);
    motorBack.resetEncoder(CTRobot::keywords::FPGA);
    motorBack.setPIDValue(P_float, I_float, D_float, 1.0/sampleRate);

    motorMid = CTRobot::motorController (h2p_lw_pwm_values_addr[1], h2p_lw_gpio1_addr, h2p_lw_adc, 6); //back motor/board
    motorMid.attachEncoder(h2p_lw_quad_addr[1], h2p_lw_quad_reset_addr, 6);
    motorMid.resetEncoder(CTRobot::keywords::FPGA);
    motorMid.setPIDValue(P_float, I_float, D_float, 1.0/sampleRate);

    motorInsertion = CTRobot::motorController (h2p_lw_pwm_values_addr[2], h2p_lw_gpio1_addr, h2p_lw_adc, 5); //back motor/board
    motorInsertion.attachEncoder(h2p_lw_quad_addr[2], h2p_lw_quad_reset_addr, 5);
    motorInsertion.resetEncoder(CTRobot::keywords::FPGA);
    motorInsertion.setPIDValue(P_float, I_float, D_float, 1.0/sampleRate);

    motorTip = CTRobot::motorController (h2p_lw_pwm_values_addr[3], h2p_lw_gpio1_addr, h2p_lw_adc, 4); //back motor/board
    motorTip.attachEncoder(h2p_lw_quad_addr[3], h2p_lw_quad_reset_addr, 4);
    motorTip.resetEncoder(CTRobot::keywords::FPGA);
    motorTip.setPIDValue(P_float, I_float, D_float, 1.0/sampleRate);
    //end initialize motors

/* set thread priorities
 */

    std::cout<<"hi"<<std::endl;
    set_realtime_priority();
    std::cout<<"bye"<<std::endl;



/*------------------------------------------------------------------------------------------------
* control loop
*------------------------------------------------------------------------------------------------*/


    auto systemStart = std::chrono::high_resolution_clock::now();
    double setPoint = 0;
    int stopCounter = 0;

    while(exit_flag == 0 || zeroed == 0) {
        //Get time since starting the program!
        // This is useful if you want to run functions such as sin(t)
        auto iterationStart = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> timeSinceStart = iterationStart - systemStart;

        if(exit_flag == 0)
            setPoint = sin(timeSinceStart.count() * 1 * M_PI)*160000;
        else {
            setPoint = sin(timeSinceStart.count() * 1 * M_PI)*160000;
            //setPoint = 0;
            stopCounter++;
        }

        if(stopCounter>2000)
            zeroed = 1;

        //Why is setPoint being converted to integer here??
        motorBack.runPID((int32_t) setPoint, CTRobot::keywords::FPGA);
        motorMid.runPID((int32_t) setPoint, CTRobot::keywords::FPGA);
        motorInsertion.runPID((int32_t) setPoint + 200000, CTRobot::keywords::FPGA);
        motorTip.runPID((int32_t) setPoint * 2, CTRobot::keywords::FPGA);


        //manage loop rate
        auto iterationEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::micro> duration = iterationEnd - iterationStart;
        std::chrono::microseconds dt_chrono((int) (std::pow(10, 6) * dt));

        if (duration >= dt_chrono * 1.0) {
            int trash = 0;
            std::cout << "overrun" << std::endl;
        }
        else {
            std::this_thread::sleep_for(dt_chrono - duration);
        }//end of managing loop rate
    } //end of control loop

    motorBack.stop();
    heartBeatThread.join();
    usleep(1000);


    return 0;
}