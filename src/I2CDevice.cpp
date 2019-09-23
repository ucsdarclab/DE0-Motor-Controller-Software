//
// Created by bunny on 8/19/19.
//

#include "DE0_Motor_Controller/I2CDevice.h"

namespace CTRobot{
    I2CSlaveDevice& I2CSlaveDevice::attachToBus(CTRobot::I2CBus *aBus) {
        aBus->addAddress(deviceAddress);
        theBus = aBus;
        return *this;
    }
}
