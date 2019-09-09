//
// Created by bunny on 8/19/19.
//

#ifndef TRYING_TEST_1_FPGAENCODER_H
#define TRYING_TEST_1_FPGAENCODER_H

#include "fpgaAddressComm.h"

namespace CTRobot{
    class fpgaEncoder{
    public:
        fpgaEncoder(unsigned char* aEncoderAddr, unsigned char* aEncoderResetAddr, uint32_t aIndex): encoderAddr(aEncoderAddr), encoderResetAddr(aEncoderResetAddr), encoderIndex(aIndex){};
        ~fpgaEncoder(){};

        int32_t readValue(){
            return (int32_t)encoderAddr.readWord();
        }

        void reset(){
            encoderResetAddr.writeWord(0);
        }

        uint32_t getIndex(){
            return encoderIndex;
        }


    protected:
        fpgaCommunication encoderAddr;
        fpgaCommunication encoderResetAddr;
        uint32_t encoderIndex;
    };

}


#endif //MOTOR_TEST_1_FPGAENCODER_H
