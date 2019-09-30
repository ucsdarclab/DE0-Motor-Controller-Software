//
// Created by bunny on 8/19/19.
//
#include "DE0_Motor_Controller/motorController.h"

namespace CTRobot{
    bool motorController::move(uint32_t aSpeed, keywords aKeyword) {
//      acquire current state and compute altered state
        uint32_t temp = motorDirection.readWord();
        temp = temp & ~(1UL<<motorIndex);
        if(aKeyword == keywords::forward)
            temp = temp | (0<<motorIndex);
        else
            temp = temp | (1<<motorIndex);

        motorDirection.writeWord(temp);
        motorPWM.writeWord(aSpeed);

        return true;
    }

    void motorController::stop() {
        motorPWM.writeWord((uint32_t)0);
    }

    motorController &motorController::attachEncoder(unsigned char *aEncoderAddr, unsigned char* encoderResetAddr, uint32_t anIndex) {
        if(motorEncoder != nullptr)
            delete(motorEncoder);
        motorEncoder = new fpgaEncoder(aEncoderAddr, encoderResetAddr, anIndex);
        resetEncoder();
        return *this;
    }
    motorController &motorController::attachEncoder(I2CBus* aBus, uint8_t anI2cAddr, uint32_t anOffset) {
        if(i2cEncoder != nullptr)
            delete(i2cEncoder);
        i2cEncoder = new AS5048b(anI2cAddr);
        i2cEncoder->attachToBus(aBus);
//        i2cEncoder->resetEncoder();
        return*this;
    }

    int32_t motorController::readEncoder() {
        if(motorEncoder == nullptr){
            throw std::runtime_error(std::string("FPGA Encoder not attached"));
        }

        return motorEncoder->readValue();
    }

    double motorController::readI2CEncoder(keywords aKeywords){
        if(i2cEncoder == nullptr){
            throw std::runtime_error(std::string("I2C Encoder not attached"));
        }

        return i2cEncoder->readAngle(aKeywords);
    }

    void motorController::resetEncoder(keywords aKeyword) {
        if(motorEncoder == nullptr && i2cEncoder == nullptr){
            throw std::runtime_error(std::string("Encoder not attached"));
        }
        if(aKeyword == keywords::FPGA)
            motorEncoder->reset();
        else
            i2cEncoder->resetEncoder();
    }

    uint32_t motorController::getIndex() {
        return motorIndex;
    }

    int32_t motorController::readADC() {
        motorADC.writeWord(0);
        return motorADC.readWord(4*motorIndex);
    }

    motorController& motorController::setPIDValue(float aP, float aI, float aD, float dt) {
        rc_filter_pid(&PIDfilter,aP,aI,aD,2*dt,dt);
        return *this;
    }

    void motorController::runPID(int32_t aDestination, keywords aKeyword, uint16_t aCap) {
        if (motorEncoder == nullptr && i2cEncoder == nullptr) {
            throw std::runtime_error(std::string("Encoder not attached"));
        }
        //ask Dmitri about scaling
        int32_t PWMOutput;
        if (aKeyword == CTRobot::keywords::FPGA)
            PWMOutput = rc_filter_march(&PIDfilter, aDestination - readEncoder()) * 2048;
        else if(aKeyword == CTRobot::keywords::I2C)
            PWMOutput = rc_filter_march(&PIDfilter, aDestination - readI2CEncoder()) * 2048;
        else
            throw std::runtime_error(std::string("wrong keywords for encoder"));
//        std::cout<<PWMOutput<<std::endl;
        auto motorDir = (PWMOutput >= 0) ? (CTRobot::keywords::forward) : (CTRobot::keywords::backward);
        PWMOutput = abs(PWMOutput);

        PWMOutput = (PWMOutput >= aCap) ? aCap : PWMOutput;

        move(PWMOutput, motorDir);

    }
}

