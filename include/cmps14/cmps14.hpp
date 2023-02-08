/**
 * cmps14.hpp
 * Author: Ethan Garnier
 */
#pragma once

#include <iostream>
#include <cstdio>
#include <cstdint>
#include <string>

// The default I2C address of the CMPS14
#define CMPS14_I2C_DEFAULT_ADDRESS 0x60

// The default Serial communication port of the CMP14
#define CMPS14_SERIAL_DEFAULT_PORT "/dev/serial0"

class cmps14
{
private:
    // Set in constructor, decides if we communicate over
    // I2C or serial with CMPS134.
    bool _i2c;

    // The I2C address being used by the CMPS14.
    // Only used if _i2c is true.
    uint16_t _i2cAddr;

    // The serial port being used by the CMPS14.
    // Only used if _i2c is false.
    std::string _serialPort;

    // Reading helper functions
    uint8_t _readByte(uint8_t reg = 0x00);

    // Writing helper functions
    uint8_t _writeByte(uint8_t data, uint8_t reg = 0x00);

public:
    cmps14(bool i2c = true);
    cmps14(uint16_t i2cAddr);
    cmps14(std::string serialPort);
    ~cmps14();

    int begin();

    int getSoftwareVersion();

    float getHeading();
};