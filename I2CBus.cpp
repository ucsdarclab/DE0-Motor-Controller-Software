//
// Created by bunny on 8/19/19.
//

#include "I2CBus.h"


namespace CTRobot{
    busTracker *busTracker::theTracker = nullptr;

    I2CBus::I2CBus(std::string I2CBusName) {
        buffer = new unsigned char[2];
        currentSlaveAddress = 0;
        busTracker *theTracker = busTracker::getInstance();
        if(!theTracker->addBus(I2CBusName)){
            throw "bus in use";
        }
        bus = open(I2CBusName.c_str(),O_RDWR);
        if(bus < 0)
            throw "cannot open bus";
    }
//    print out addresses currently on bus.
    void I2CBus::I2C_scan() {
        uint8_t value;
        for(int addr = 1;addr<127;addr += 1){
            if(ioctl(bus,I2C_SLAVE,addr) >= 0){
                if(read(bus,&value,sizeof(value)) == 1)
                    std::cout<<std::hex<<addr<<std::endl;
            }
        }
    }

    void I2CBus::bus_closd() {
        close(bus);
    }

//    begin transmission to an address
    bool I2CBus::beginTransmission(uint8_t anAddress) {
        if(ioctl(bus,I2C_SLAVE,anAddress)<0)
            return false;
        if(addressList.find(anAddress) == addressList.end())
            throw std::runtime_error(std::string("add address to bus first"));
        currentSlaveAddress = anAddress;
        return true;
    }

    bool I2CBus::readFromSlave(uint8_t &anValue) {
        return read(bus, &anValue, sizeof(anValue)) != 0;
    }

    bool I2CBus::readFromSlave(uint16_t &anValue) {
        return read(bus, &anValue, sizeof(anValue)) != 0;
    }

    bool I2CBus::read14(uint16_t &anValue) {
        return read(bus, &anValue, 14) != 0;
    }

//    returns pointer, watch out for leaks
    unsigned char* I2CBus::readMult(uint32_t aDigits) {
        auto data = new unsigned char[2];
        read(bus,data,aDigits);
        return buffer;
    }

//---------currently might not be working corrently, not used-----
    bool I2CBus::writeToSLave(uint8_t anValue[2]) {
        return write(bus, anValue, 2) == 2;
    }

    bool I2CBus::writeToSLave(uint8_t anValue) {
        return write(bus, &anValue, sizeof(anValue)) == sizeof(anValue);
    }
//----------------------------------------------------------------
    I2CBus& I2CBus::addAddress(uint8_t anAddress) {
        if (addressList.find(anAddress) == addressList.end()) {
            addressList.insert(anAddress);
            return *this;
        } else
            throw std::runtime_error(std::string("duplicated I2C address"));
    }
}
