/**
 * cmps14.cpp
 * Author: Ethan Garnier
 */
#include "cmps14/cmps14.hpp"

// https://www.kernel.org/doc/Documentation/i2c/smbus-protocol
// Required for I2C communication
// #include <linux/i2c-dev.h>
// #include <i2c/smbus.h>

// TODO: Link these in CMakeLists.txt
// Required for serial communication
#include <wiringPi.h>
#include <wiringSerial.h>
#include <wiringPiI2C.h>

// Register addresses on the CMPS14
#define CMPS14_BEARING_HIGH 0x02
#define CMPS14_BEARING_LOW 0x03
#define CMPS14_PITCH 0x04
#define CMPS14_ROLL 0x05

#define CMPS14_MAGNETOMETER_X_HIGH 0x06
#define CMPS14_MAGNETOMETER_X_LOW 0x07
#define CMPS14_MAGNETOMETER_Y_HIGH 0x08
#define CMPS14_MAGNETOMETER_Y_LOW 0x09
#define CMPS14_MAGNETOMETER_Z_HIGH 0x0A
#define CMPS14_MAGNETOMETER_Z_LOW 0x0B

#define CMPS14_ACCELEROMETER_X_HIGH 0x0C
#define CMPS14_ACCELEROMETER_X_LOW 0x0D
#define CMPS14_ACCELEROMETER_Y_HIGH 0x0E
#define CMPS14_ACCELEROMETER_Y_LOW 0x0F
#define CMPS14_ACCELEROMETER_Z_HIGH 0x10
#define CMPS14_ACCELEROMETER_Z_LOW 0x11

#define CMPS14_GYROSCOPE_X_HIGH 0x12
#define CMPS14_GYROSCOPE_X_LOW 0x13
#define CMPS14_GYROSCOPE_Y_HIGH 0x14
#define CMPS14_GYROSCOPE_Y_LOW 0x15
#define CMPS14_GYROSCOPE_Z_HIGH 0x16
#define CMPS14_GYROSCOPE_Z_LOW 0x17

#define CMPS14_ROLL_HIGH 0x1C
#define CMPS14_ROLL_LOW 0x1D

#define CMPS14_CAL_STATE 0x1E

#define CMPS14_SERIAL_BAUD_RATE 9600

int cmps14_fd;

cmps14::cmps14(bool i2c, uint16_t i2cAddr, std::string serialPort)
{
    _i2c = i2c;
    _i2cAddr = i2cAddr;
    _serialPort = serialPort;
}

int cmps14::begin()
{
    // If _i2c is true, communicate over i2c, otherwise communicate over serial port
    cmps14_fd = (_i2c) ? wiringPiI2CSetup(_i2cAddr) : serialOpen(_serialPort.c_str(), CMPS14_SERIAL_BAUD_RATE);
    if (cmps14_fd == -1)
    {
        std::cerr << "Unable to initialize communication with CMPS14" << std::endl;
        return -1;
    }

    // Initialize WiringPi
    if (wiringPiSetup() == -1)
    {
        std::cerr << "Unable to start WiringPi" << std::endl;
        return -1;
    }

    return 1;
}

cmps14::~cmps14()
{
    // Close the serial file descriptor if needed
    if (!_i2c)
        serialClose(cmps14_fd);
}