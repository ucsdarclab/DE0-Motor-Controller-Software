//
// Created by bunny on 8/19/19.
//

#ifndef MOTOR_TEST_1_FPGAADDRESSCOMM_H
#define MOTOR_TEST_1_FPGAADDRESSCOMM_H

#include <cstdint>
#include <iostream>
#include <map>
#include "fpga_headers/hps_0.h"
#include "fpga_headers/hwlib.h"
#include "fpga_headers/socal.h"
#include "fpga_headers/hps.h"
#include "fpga_headers/alt_gpio.h"


namespace CTRobot{
    enum class keywords{
        forward,backward,I2C,FPGA,degree,radian,
    };
//    fool proof dictionary for motor direction
    static std::map<keywords, uint32_t > controlKeywords = {
            std::make_pair(keywords::forward,   0),
            std::make_pair(keywords::backward,  1),
            std::make_pair(keywords::I2C,       2),
            std::make_pair(keywords::FPGA,      3),
            std::make_pair(keywords::degree,      4),
            std::make_pair(keywords::radian,      5)
    };
//  fpga communication is the base class for wrapper of an fpga address.
//  further functions of the fpga library should inherent from this class or use it as members
    class fpgaCommunication{
    public:
        fpgaCommunication(unsigned char* anAddr):memAddress(anAddr){};
        fpgaCommunication(fpgaCommunication const &aCommunication):memAddress(aCommunication.memAddress){};
        ~fpgaCommunication(){}

        //-------------write functions--------------
        bool writeByte(uint8_t aByte){                  //write a byte to the memory address
            alt_write_byte(memAddress, aByte);
        }
        bool writeTwoBytes(uint16_t aTwoByte){           //write two bytes
            alt_write_hword(memAddress, aTwoByte);
        }
        void writeWord(uint32_t aWord){                 //write a word(4 bytes)
            alt_write_word(memAddress, aWord);
        }
        bool writeTwoWords(uint64_t aTwoWord){          //write two words
            alt_write_dword(memAddress, aTwoWord);
        }

        //-------------read functions--------------
        uint8_t readByte(){
            return alt_read_byte(memAddress);
        }
        uint16_t readTwoBytes(){
            return alt_read_hword(memAddress);
        }
        int32_t readWord(int32_t offset = 0){
            return alt_read_word(memAddress + offset);
        }
        uint64_t readTwoWords(){
            return alt_read_dword(memAddress);
        }


    protected:
        unsigned char* memAddress;
    };

//    heartbeat function for the stuff.
    class fpgaHeartBeat: public fpgaCommunication{
    public:
        fpgaHeartBeat(unsigned char *anAddr) : fpgaCommunication(anAddr),currentState(0){};

        ~fpgaHeartBeat(){}

        void changeState(){
            alt_write_word(memAddress,currentState);
            currentState = currentState xor (uint32_t)1;
        }

    protected:
        uint32_t currentState;
    };

}

#endif //MOTOR_TEST_1_FPGAADDRESSCOMM_H
