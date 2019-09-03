//
// Created by bunny on 8/19/19.
//

#include "AS5048b.h"

namespace CTRobot{

    uint8_t  AS5048b::addressRegR() {
        return readReg8(AS5048B_ADDR_REG);
    }

    uint16_t AS5048b::magnitudeR() {
        return readReg16(AS5048B_MAGNMSB_REG);
    }


    uint16_t AS5048b::readAngleRaw() {
        currentAngleR = readReg16(AS5048B_ANGLMSB_REG);
        MSB = (uint8_t)((currentAngleR & 0xFF00) >> 8);
        LSB = (uint8_t)(currentAngleR & 0x00FF);
        currentAngleR = ((uint16_t)MSB<<6)|((uint16_t)LSB & 0x3f);

        return currentAngleR ;//- offset;// - offset;
    }

    double AS5048b::readAngle(CTRobot::keywords aKeywords) {
        readAngleRaw();
        if(aKeywords == keywords::degree){
            currentAngle = (currentAngleR >= offset) ? (currentAngleR-offset)/AS5048B_RESOLUTION*360 :
                           (AS5048B_RESOLUTION - offset + currentAngleR)/AS5048B_RESOLUTION*360;
            currentAngle = (currentAngle <= 180) ? (currentAngle) : (currentAngle - 360);
        }else if(aKeywords == keywords::radian){
            currentAngle = (currentAngleR >= offset) ? (currentAngleR-offset)/AS5048B_RESOLUTION*6.283384 :
                           (AS5048B_RESOLUTION - offset + currentAngleR)/AS5048B_RESOLUTION*6.283384;
            
        }else
            throw std::runtime_error(std::string("wrong keyword for radian or degree"));

        return currentAngle;
    }

    uint16_t AS5048b::readAngleAveraged(CTRobot::keywords aKeywords) {
        return 0.8*currentAngle + 0.2*readAngle(aKeywords);
    }


    AS5048b& AS5048b::resetEncoder() {
//        hardware reset through chip seems to not work, using software reset
//        offset = readAngle();
//        writeReg8(AS5048B_ZEROMSB_REG,((uint8_t)0x00));
//        writeReg8(AS5048B_ZEROLSB_REG,((uint8_t)0x00));
//
//        uint16_t ret = readReg16(AS5048B_ANGLMSB_REG);
//
//        writeReg8(AS5048B_ZEROMSB_REG,((uint8_t)ret >> 6));
//        writeReg8(AS5048B_ZEROLSB_REG,((uint8_t)ret & 0x3f));
        offset = readAngleRaw();
        return*this;
    }


    //private functions for trad and write;

    void AS5048b::writeReg8(uint8_t aRegister, uint8_t anInput) {

        if(!theBus->beginTransmission(deviceAddress))
            throw std::runtime_error(std::string("cannot begin transmission"));

        uint8_t temp[2]; temp[0] = aRegister;temp[1] = anInput;
        theBus->writeToSLave(temp);
    }

    uint8_t AS5048b::readReg8(uint8_t anRegister){
        uint8_t readValue;
        if(!theBus->beginTransmission(deviceAddress))
            throw std::runtime_error(std::string("cannot begin transmission"));

        theBus->writeToSLave(anRegister);
        theBus->readFromSlave(readValue);

        return readValue;
    }

    uint16_t AS5048b::readReg16(uint8_t anRegister) {
        uint16_t readValue;
        uint8_t readValueTemp[2];

        if(!theBus->beginTransmission(deviceAddress))
            throw std::runtime_error(std::string("cannot begin transmission"));

        theBus->writeToSLave(anRegister);

        theBus->readFromSlave(readValue);

        return readValue;
    }



    void AS5048b::zeroRegW(uint16_t aValue) {
        writeReg8(AS5048B_ZEROMSB_REG,(uint8_t) (aValue >> 6));
        writeReg8(AS5048B_ZEROLSB_REG,(uint8_t) (aValue & 0x3F));
    }

}