//
// Created by bunny on 8/19/19.
//

#ifndef MOTOR_TEST_1_I2CDEVICE_H
#define MOTOR_TEST_1_I2CDEVICE_H

#include <cstdint>
#include "I2CBus.h"

namespace CTRobot{
    class I2CSlaveDevice{
    public:
        I2CSlaveDevice(uint8_t anAddress): deviceAddress(anAddress){}
        virtual ~I2CSlaveDevice(){}
        I2CSlaveDevice &attachToBus(I2CBus *aBus);


    protected:
        uint8_t deviceAddress;
        I2CBus *theBus;
    };

}

#endif //MOTOR_TEST_1_I2CDEVICE_H
