//
// Created by bunny on 8/19/19.
//

#ifndef MOTOR_TEST_1_I2CBUS_H
#define MOTOR_TEST_1_I2CBUS_H

#include <string>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cstdint>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unordered_set>
#include <iostream>
#include <stdexcept>

#include "fpga_headers/hwlib.h"

namespace CTRobot {
//------tracker to maintain single access to a I2C bus, class is singleton.
    class busTracker{
    protected:
        std::unordered_set<std::string>   trackList;
        static busTracker                 *theTracker;

        busTracker()= default;

    public:
//------return a singleton instance of the bus tracker.
        static busTracker *getInstance(){
            if(!theTracker)
                theTracker = new busTracker;
            return theTracker;
        }

        static void deleteTracker(){
            delete theTracker;
        }

        bool addBus(std::string &aBus){
            if(trackList.find((aBus)) == trackList.end())
                trackList.insert(aBus);
            else
                return false;
            return true;
        }

        ~busTracker(){
            std::cout <<"tracker deleted"<<std::endl;

        }
    };

//class of I2C bus
    class I2CBus{
    public:
        I2CBus(std::string I2CBusName);

        ~I2CBus(){

            close(bus);
            std::cout<<"bus closed"<<std::endl;
            busTracker::deleteTracker();
            delete[] buffer;
        }

        I2CBus& operator= (I2CBus &aCopy){bus = aCopy.bus;return*this;}

        void I2C_scan();
        void bus_closd();
        bool beginTransmission(uint8_t anAddress);
        bool readFromSlave(uint8_t &anValue);
        bool readFromSlave(uint16_t &anValue);
        bool read14(uint16_t &anValue);
        unsigned char* readMult(uint32_t aDigits);
        bool writeToSLave(uint8_t anValue);
        bool writeToSLave(uint8_t anValue[2]);
        I2CBus& addAddress(uint8_t anAaddress);


    protected:
        int bus;
        uint8_t currentSlaveAddress;
        std::unordered_set<uint8_t> addressList;
        unsigned char* buffer;
    };

}


#endif //MOTOR_TEST_1_I2CBUS_H
