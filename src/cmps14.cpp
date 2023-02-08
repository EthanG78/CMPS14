/**
 * cmps14.cpp
 * Author: Ethan Garnier
 */
#include "cmps14/cmps14.hpp"

#include <chrono>
#include <thread>

// Required for serial/i2c communication
#include <wiringPi.h>
#include <wiringSerial.h>
#include <wiringPiI2C.h>

// TODO: REORGANIZE DEFINES

// CMPS14 I2C Registers
#define CMPS14_HEADING_HIGH 0x02
#define CMPS14_HEADING_LOW 0x03
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

// CMPS14 Serial commands
#define CMPS14_SERIAL_BAUD_RATE 9600

#define CMPS14_SVER_CMD 0x11
#define CMPS14_HEADING_8BIT_CMD 0x12
#define CMPS14_HEADING_16BIT_CMD 0x13
#define CMPS14_PITCH_CMD 0x14
#define CMPS14_ROLL_90_CMD 0x15
#define CMPS14_ROLL_180_CMD 0x26
#define CMPS14_MAG_RAW_CMD 0x19
#define CMPS14_ACCEL_RAW_CMD 0x20
#define CMPS14_GYRO_RAW_CMD 0x21
#define CMPS14_ALL_ORIENT_CMD 0x23
#define CMPS14_CALIB_STATE_CMD 0x24
#define CMPS14_CHANGE_CALIB_CONFIG_BYTE1_CMD 0x98
#define CMPS14_CHANGE_CALIB_CONFIG_BYTE2_CMD 0x95
#define CMPS14_CHANGE_CALIB_CONFIG_BYTE3_CMD 0x99
#define CMPS14_STORE_CALIB_BYTE1_CMD 0xF0
#define CMPS14_STORE_CALIB_BYTE2_CMD 0xF5
#define CMPS14_STORE_CALIB_BYTE3_CMD 0xF6
#define CMPS14_DELETE_CALIB_BYTE1_CMD 0xE0
#define CMPS14_DELETE_CALIB_BYTE2_CMD 0xE5
#define CMPS14_DELETE_CALIB_BYTE3_CMD 0xE2
#define CMPS14_BAUD_19200 0xA0
#define CMPS14_BAUD_38400 0xA1

int cmps14_fd;

uint8_t cmps14::_readByte(uint8_t reg)
{
    return static_cast<uint8_t>((_i2c) ? wiringPiI2CReadReg8(cmps14_fd, reg) : serialGetchar(cmps14_fd));
}

int8_t cmps14::_readSignedByte(uint8_t reg)
{
    return static_cast<int8_t>((_i2c) ? wiringPiI2CReadReg8(cmps14_fd, reg) : serialGetchar(cmps14_fd));
}

// TODO: Figure out return values here
uint8_t cmps14::_writeByte(uint8_t data, uint8_t reg)
{
    if (_i2c)
    {
        // No clue what this returns
        wiringPiI2CWriteReg8(cmps14_fd, reg, data);
    }
    else
    {
        serialPutchar(cmps14_fd, data);
    }

    return 0x00;
}

cmps14::cmps14(bool i2c)
{
    _i2c = i2c;
    _i2cAddr = CMPS14_I2C_DEFAULT_ADDRESS;
    _serialPort = CMPS14_SERIAL_DEFAULT_PORT;
}

cmps14::cmps14(uint16_t i2cAddr)
{
    _i2c = true;
    _i2cAddr = i2cAddr;
}

cmps14::cmps14(std::string serialPort)
{
    _i2c = false;
    _serialPort = serialPort;
}

cmps14::~cmps14()
{
    // Close the serial file descriptor if needed
    if (!_i2c)
        serialClose(cmps14_fd);
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

    // debugging
    std::cout << "CMPS14 IMU INITIALIZED" << std::endl;

    return 1;
}

int cmps14::getSoftwareVersion()
{
    int ver;
    if (_i2c)
    {
        ver = static_cast<int>(_readByte(0x00));
    }
    else
    {
        _writeByte(CMPS14_SVER_CMD);
        while (!serialDataAvail(cmps14_fd))
        {
        }

        ver = static_cast<int>(_readByte());
    }

    return ver;
}

std::vector<int> cmps14::getCalibrationStatus()
{
    uint8_t state;
    if (_i2c)
    {
        state = _readByte(CMPS14_CAL_STATE);
    }
    else
    {
        _writeByte(CMPS14_CALIB_STATE_CMD);
        while (!serialDataAvail(cmps14_fd))
        {
        }

        state = _readByte();
    }

    std::vector<int> calStatus{
        static_cast<int>(state & 0x03),      // magnetometer calibration
        static_cast<int>(state & 0x0C >> 2), // accelerometer calibration
        static_cast<int>(state & 0x30 >> 4), // gyroscope calibration
        static_cast<int>(state & 0xC0 >> 6)  // system calibration
    };

    return calStatus;
}

float cmps14::getHeading()
{
    uint8_t headingMsb;
    uint8_t headingLsb;

    if (_i2c)
    {
        headingMsb = _readByte(CMPS14_HEADING_HIGH);
        headingLsb = _readByte(CMPS14_HEADING_LOW);
    }
    else
    {
        _writeByte(CMPS14_HEADING_16BIT_CMD);
        while (!serialDataAvail(cmps14_fd))
        {
        }
        headingMsb = _readByte();

        while (!serialDataAvail(cmps14_fd))
        {
        }
        headingLsb = _readByte();
    }

    uint16_t heading = ((uint16_t)headingMsb << 8) | headingLsb;

    // Compass heading 16 bit, i.e. 0-3599, representing 0-359.9 degrees, therefore
    // need to extract last digit as decimal in tens place.
    float decimal = static_cast<float>(heading % 10) / 10;
    return static_cast<float>(heading / 10) + decimal;
}

float cmps14::getPitch()
{
    int8_t pitch;
    if (_i2c)
    {
        pitch = _readSignedByte(CMPS14_PITCH);
    }
    else
    {
        _writeByte(CMPS14_PITCH_CMD);
        while (!serialDataAvail(cmps14_fd))
        {
        }
        pitch = _readSignedByte();
    }

    return static_cast<float>(pitch);
}

// BROKEN??
float cmps14::getRoll()
{
    // WIP: MATH IS WRONG RIGHT NOW ON HIGHER RESOLUTION ROLL
    /*int8_t rollMsb;
    int8_t rollLsb;

    if (_i2c)
    {
        rollMsb = _readSignedByte(CMPS14_ROLL_HIGH);
        rollLsb = _readSignedByte(CMPS14_ROLL_LOW);
    }
    else
    {
        _writeByte(CMPS14_ROLL_180_CMD);
        while (!serialDataAvail(cmps14_fd))
        {
        }
        rollMsb = _readSignedByte();

        while (!serialDataAvail(cmps14_fd))
        {
        }
        rollLsb = _readSignedByte();
    }

    int16_t roll = (static_cast<int16_t>(rollMsb) << 8) | rollLsb;

    // 16 bit roll angle for +/- 180 from the horizontal plane.
    // Value is in tenths of degrees (range of +/- 1800), thus
    // need to extract last digit as decimal in tens place
    float decimal = static_cast<float>(roll % 10) / 10;
    if (roll < 0)
        decimal *= -1;
    return static_cast<float>(roll / 10) + decimal;*/

    int8_t roll;
    if (_i2c)
    {
        roll = _readSignedByte(CMPS14_ROLL);
    }
    else
    {
        _writeByte(CMPS14_ROLL_90_CMD);
        while (!serialDataAvail(cmps14_fd))
        {
        }
        roll = _readSignedByte();
    }

    return static_cast<float>(roll);
}