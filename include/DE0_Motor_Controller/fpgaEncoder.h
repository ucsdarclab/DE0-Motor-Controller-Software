//
// Created by bunny on 8/19/19.
//

#ifndef TRYING_TEST_1_FPGAENCODER_H
#define TRYING_TEST_1_FPGAENCODER_H

#include "fpgaAddressComm.h"

namespace CTRobot{
    class fpgaEncoder{
    public:
        fpgaEncoder(unsigned char* aEncoderAddr, unsigned char* aEncoderResetAddr, uint32_t aIndex): encoderAddr(aEncoderAddr), encoderResetAddr(aEncoderResetAddr),
        encoderIndex(aIndex), offset(0){};
        ~fpgaEncoder(){};

        int32_t readValue(){
            return (int32_t)encoderAddr.readWord() - offset;
        }

        void reset(){
//            uint32_t temp = encoderResetAddr.readWord();
//            temp = temp & ~(0b01<<encoderIndex);
//            encoderResetAddr.writeWord(0xffffffff); //<< (7 - encoderIndex));
                offset = encoderResetAddr.readWord();
        }

        uint32_t getIndex(){
            return encoderIndex;
        }


    protected:
        fpgaCommunication encoderAddr;
        fpgaCommunication encoderResetAddr;
        uint32_t encoderIndex;
        int32_t offset;
    };

}


#endif //MOTOR_TEST_1_FPGAENCODER_H
