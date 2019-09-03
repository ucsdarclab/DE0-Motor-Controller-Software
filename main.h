//
// Created by bunny on 8/20/19.
//

#ifndef MOTOR_TEST_1_MAIN_H
#define MOTOR_TEST_1_MAIN_H

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

extern "C"{
#include <rc/math/filter.h>
}

#include "fpga_headers/hps_0.h"
#include "fpga_headers/hwlib.h"
#include "fpga_headers/socal.h"
#include "fpga_headers/hps.h"
#include "fpga_headers/alt_gpio.h"
#include "motorController.h"
#include "fpgaAddressComm.h"
#include "fpgaEncoder.h"
#include "AS5048b.h"


#define P_float 0.0005
#define I_float 0.00005
#define D_float 0.0000006
#define SAMPLE_RATE 1000
#define SYNC_TOLERANCE 10
//#define dt (1.0/(float)SAMPLE_RATE)
#define interval_time_us ((int)(dt * 1000000))
#define update_dt 0.01
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )
#define MAX_TRAVEL_RANGE 10000000
#define MAX_CURRENT 2.5 //amps



#endif //MOTOR_TEST_1_MAIN_H
