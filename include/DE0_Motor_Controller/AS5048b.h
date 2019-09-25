//
// Created by bunny on 8/19/19.
//

#ifndef MOTOR_TEST_1_AS5048B_H
#define MOTOR_TEST_1_AS5048B_H

#include <cmath>
#include "I2CDevice.h"
#include "fpgaAddressComm.h"//only for keywords

// Default addresses for AS5048B
#define AS5048_ADDRESS 0x40 // 0b10000 + ( A1 & A2 to GND)
#define AS5048B_PROG_REG 0x03
#define AS5048B_ADDR_REG 0x15
#define AS5048B_ZEROMSB_REG 0x16 //bits 0..7
#define AS5048B_ZEROLSB_REG 0x17 //bits 0..5
#define AS5048B_GAIN_REG 0xFA
#define AS5048B_DIAG_REG 0xFB
#define AS5048B_MAGNMSB_REG 0xFC //bits 0..7
#define AS5048B_MAGNLSB_REG 0xFD //bits 0..5
#define AS5048B_ANGLMSB_REG 0xFE //bits 0..7
#define AS5048B_ANGLLSB_REG 0xFF //bits 0..5
#define AS5048B_RESOLUTION 16384.0 //14 bits

// Moving Exponential Average on angle - beware heavy calculation for some Arduino boards
// This is a 1st order low pass filter
// Moving average is calculated on Sine et Cosine values of the angle to provide an extrapolated accurate angle value.
#define EXP_MOVAVG_N 5	//history length impact on moving average impact - keep in mind the moving average will be impacted by the measurement frequency too
#define EXP_MOVAVG_LOOP 1 //number of measurements before starting mobile Average - starting with a simple average - 1 allows a quick start. Value must be 1 minimum

//unit consts - just to make the units more readable
#define U_RAW 1
#define U_TRN 2
#define U_DEG 3
#define U_RAD 4
#define U_GRAD 5
#define U_MOA 6
#define U_SOA 7
#define U_MILNATO 8
#define U_MILSE 9
#define U_MILRU 10

namespace CTRobot{
    class AS5048b: public I2CSlaveDevice{
    public:
        AS5048b(uint8_t anAddress, int16_t aOffset = 0): I2CSlaveDevice(anAddress), offset(aOffset),MSB(0),LSB(0),currentAngle(0),
        currentAngleR(0),previousAngle(0){
        }
        ~AS5048b(){
        }

        uint8_t     addressRegR();
        uint16_t    magnitudeR();
        uint16_t    readAngleRaw();
        double    readAngle(keywords aKeywords = keywords::degree);
        uint16_t    readAngleAveraged(CTRobot::keywords aKeywords);// 0.2*current + 0.8 previous

        AS5048b&    resetEncoder();
        void        writeReg8(uint8_t aRegister, uint8_t anInput);
        void        zeroRegW(uint16_t aValue);

    private:
        uint8_t readReg8(uint8_t anRegister);
        uint16_t readReg16(uint8_t anRegister);
        int16_t offset;
        int16_t currentAngleR;
        double previousAngle;
        double currentAngle;
        uint8_t MSB;
        uint8_t LSB;
    };
}


#endif //MOTOR_TEST_1_AS5048B_H
