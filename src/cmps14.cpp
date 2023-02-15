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
#define CMPS14_PITCH_HIGH 0x1A
#define CMPS14_PITCH_LOW 0x1B
#define CMPS14_ROLL 0x05
#define CMPS14_ROLL_HIGH 0x1C
#define CMPS14_ROLL_LOW 0x1D
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
#define CMPS14_CAL_STATE 0x1E

// CMPS14 Serial commands
#define CMPS14_SERIAL_BAUD_RATE 9600

#define CMPS14_SVER_CMD 0x11
#define CMPS14_HEADING_8BIT_CMD 0x12
#define CMPS14_HEADING_16BIT_CMD 0x13
#define CMPS14_PITCH_90_CMD 0x14
#define CMPS14_PITCH_180_CMD 0x27
#define CMPS14_ROLL_90_CMD 0x15
#define CMPS14_ROLL_180_CMD 0x26
#define CMPS14_MAG_RAW_CMD 0x19
#define CMPS14_ACCEL_RAW_CMD 0x20
#define CMPS14_GYRO_RAW_CMD 0x21
#define CMPS14_ALL_ORIENT_CMD 0x23
#define CMPS14_CALIB_STATE_CMD 0x24
#define CMPS14_BAUD_19200 0xA0
#define CMPS14_BAUD_38400 0xA1
#define CMPS14_OK 0x55

// Calibration configuration commands
#define CMPS14_CHANGE_CALIB_CONFIG_BYTE1_CMD 0x98
#define CMPS14_CHANGE_CALIB_CONFIG_BYTE2_CMD 0x95
#define CMPS14_CHANGE_CALIB_CONFIG_BYTE3_CMD 0x99
#define CMPS14_STORE_CALIB_BYTE1_CMD 0xF0
#define CMPS14_STORE_CALIB_BYTE2_CMD 0xF5
#define CMPS14_STORE_CALIB_BYTE3_CMD 0xF6
#define CMPS14_DELETE_CALIB_BYTE1_CMD 0xE0
#define CMPS14_DELETE_CALIB_BYTE2_CMD 0xE5
#define CMPS14_DELETE_CALIB_BYTE3_CMD 0xE2

// Helper that waits for serial data to be
// available before reading it. Cleans up
// serial code.
uint8_t cmps14::_readSerialByte()
{
    while (!serialDataAvail(_cmps14Fd))
    {
    }
    return serialGetchar(_cmps14Fd);
}

uint8_t cmps14::_readByte(uint8_t reg)
{
    return static_cast<uint8_t>((_i2c) ? wiringPiI2CReadReg8(_cmps14Fd, reg) : _readSerialByte());
}

int8_t cmps14::_readSignedByte(uint8_t reg)
{
    return static_cast<int8_t>((_i2c) ? wiringPiI2CReadReg8(_cmps14Fd, reg) : _readSerialByte());
}

// TODO: Figure out return values here
uint8_t cmps14::_writeByte(uint8_t data, uint8_t reg)
{
    if (_i2c)
    {
        // No clue what this returns
        wiringPiI2CWriteReg8(_cmps14Fd, reg, data);
    }
    else
    {
        serialPutchar(_cmps14Fd, data);
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
        serialClose(_cmps14Fd);
}

int cmps14::begin()
{
    // If _i2c is true, communicate over i2c, otherwise communicate over serial port
    _cmps14Fd = (_i2c) ? wiringPiI2CSetup(_i2cAddr) : serialOpen(_serialPort.c_str(), CMPS14_SERIAL_BAUD_RATE);
    if (_cmps14Fd == -1)
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
        ver = static_cast<int>(_readByte());
    }

    return ver;
}

int cmps14::toggleBackgroundCal(bool enable)
{
    uint8_t setupByte = (enable) ? 0x97 : 0x80;

    if (_i2c)
    {
        _writeByte(CMPS14_CHANGE_CALIB_CONFIG_BYTE1_CMD, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(21));
        _writeByte(CMPS14_CHANGE_CALIB_CONFIG_BYTE2_CMD, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(21));
        _writeByte(CMPS14_CHANGE_CALIB_CONFIG_BYTE3_CMD, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(21));
        _writeByte(setupByte, 0x00);
    }
    else
    {
        _writeByte(CMPS14_CHANGE_CALIB_CONFIG_BYTE1_CMD);
        if (_readByte() != CMPS14_OK)
            return -1;
        _writeByte(CMPS14_CHANGE_CALIB_CONFIG_BYTE2_CMD, 0x00);
        if (_readByte() != CMPS14_OK)
            return -1;
        _writeByte(CMPS14_CHANGE_CALIB_CONFIG_BYTE3_CMD, 0x00);
        if (_readByte() != CMPS14_OK)
            return -1;
        _writeByte(setupByte);
    }

    return 1;
}

int cmps14::storeCalProfile()
{
    if (_i2c)
    {
        _writeByte(CMPS14_STORE_CALIB_BYTE1_CMD, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(21));
        _writeByte(CMPS14_STORE_CALIB_BYTE2_CMD, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(21));
        _writeByte(CMPS14_STORE_CALIB_BYTE3_CMD, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(21));
    }
    else
    {
        _writeByte(CMPS14_STORE_CALIB_BYTE1_CMD);
        if (_readByte() != CMPS14_OK)
            return -1;
        _writeByte(CMPS14_STORE_CALIB_BYTE2_CMD);
        if (_readByte() != CMPS14_OK)
            return -1;
        _writeByte(CMPS14_STORE_CALIB_BYTE3_CMD);
        if (_readByte() != CMPS14_OK)
            return -1;
    }

    return 1;
}

int cmps14::eraseCalProfile()
{
    if (_i2c)
    {
        _writeByte(CMPS14_DELETE_CALIB_BYTE1_CMD, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(21));
        _writeByte(CMPS14_DELETE_CALIB_BYTE2_CMD, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(21));
        _writeByte(CMPS14_DELETE_CALIB_BYTE3_CMD, 0x00);
    }
    else
    {
        _writeByte(CMPS14_DELETE_CALIB_BYTE1_CMD);
        if (_readByte() != CMPS14_OK)
            return -1;
        _writeByte(CMPS14_DELETE_CALIB_BYTE2_CMD);
        if (_readByte() != CMPS14_OK)
            return -1;
        _writeByte(CMPS14_DELETE_CALIB_BYTE3_CMD);
        if (_readByte() != CMPS14_OK)
            return -1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(301));

    return 1;
}

// NOTE:
// According to the CMPS14 documentation:
//      "Please note the Gyro feedback does not currently work,
//       this is a bug in the sensor itself that is
//       scheduled to be fixed."
//
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
        state = _readByte();
    }

    std::vector<int> calStatus{
        static_cast<int>(state & 0x03),        // magnetometer calibration
        static_cast<int>((state >> 2) & 0x03), // accelerometer calibration
        static_cast<int>((state >> 4) & 0x03), // gyroscope calibration
        static_cast<int>((state >> 6) & 0x03)  // system calibration
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
        headingMsb = _readByte();
        headingLsb = _readByte();
    }

    uint16_t heading = (static_cast<uint16_t>(headingMsb) << 8) | headingLsb;

    // Compass heading 16 bit, i.e. 0-3599, representing 0-359.9 degrees, therefore
    // need to extract last digit as decimal in tens place.
    float decimal = static_cast<float>(heading % 10) / 10;
    return static_cast<float>(heading / 10) + decimal;
}

float cmps14::getPitch()
{
    // If the CMPS14 software version is less than 5, we do
    // not have access to 16-bit pitch values. We need to check.

    /*if (_i2c)
    {
        if (static_cast<int>(_readByte(0x00)) >= 5)
        {
            int8_t pitchMsb = _readSignedByte(CMPS14_PITCH_HIGH);
            int8_t pitchLsb = _readSignedByte(CMPS14_PITCH_LOW);

            int16_t pitch = (static_cast<int16_t>(pitchMsb) << 8) | pitchLsb;

            // Value is in tenths of degrees (range of +/- 900). Have to extract last
            // digit as decimal in tenths place
            float decimal = static_cast<float>(pitch % 10) / 10;
            if (pitch < 0)
                decimal *= -1.0;
            return static_cast<float>(pitch / 10) + decimal;
        }
        else
        {
            return static_cast<float>(_readSignedByte(CMPS14_PITCH));
        }
    }
    else
    {
        _writeByte(CMPS14_SVER_CMD);
        if (static_cast<int>(_readByte()) >= 5)
        {

            _writeByte(CMPS14_PITCH_180_CMD);
            int8_t pitchMsb = _readSignedByte();
            int8_t pitchLsb = _readSignedByte();

            int16_t pitch = (static_cast<int16_t>(pitchMsb) << 8) | pitchLsb;

            // Value is in tenths of degrees (range of +/- 900). Have to extract last
            // digit as decimal in tenths place
            float decimal = static_cast<float>(pitch % 10) / 10;
            if (pitch < 0)
                decimal *= -1.0;
            return static_cast<float>(pitch / 10) + decimal;
        }
        else
        {
            _writeByte(CMPS14_PITCH_90_CMD);
            return static_cast<float>(_readSignedByte());
        }
    }*/

    int8_t pitch;
    if (_i2c)
    {
        pitch = _readSignedByte(CMPS14_PITCH);
    }
    else
    {
        _writeByte(CMPS14_PITCH_90_CMD);
        pitch = _readSignedByte();
    }

    return static_cast<float>(pitch);
}

// BROKEN??
float cmps14::getRoll()
{
    // WIP: MATH IS WRONG RIGHT NOW ON HIGHER RESOLUTION ROLL
    int8_t rollMsb;
    int8_t rollLsb;

    if (_i2c)
    {
        rollMsb = _readSignedByte(CMPS14_ROLL_HIGH);
        rollLsb = _readSignedByte(CMPS14_ROLL_LOW);
    }
    else
    {
        _writeByte(CMPS14_ROLL_180_CMD);
        rollMsb = _readSignedByte();
        rollLsb = _readSignedByte();
    }

    int16_t roll = (static_cast<int16_t>(rollMsb) << 8) | rollLsb;

    // 16 bit roll angle for +/- 180 from the horizontal plane.
    // Value is in tenths of degrees (range of +/- 1800), thus
    // need to extract last digit as decimal in tens place
    float decimal = static_cast<float>(roll % 10) / 10;
    if (roll < 0)
        decimal *= -1;
    return static_cast<float>(roll / 10) + decimal;

    /*int8_t roll;
    if (_i2c)
    {
        roll = _readSignedByte(CMPS14_ROLL);
    }
    else
    {
        _writeByte(CMPS14_ROLL_90_CMD);
        roll = _readSignedByte();
    }

    return static_cast<float>(roll);*/
}

// WIP:
std::vector<float> cmps14::getOrientation()
{
    uint16_t heading;
    uint8_t headingMsb, headingLsb;
    int8_t pitch, roll;

    if (_i2c)
    {
        // TODO:
    }
    else
    {
        _writeByte(CMPS14_ALL_ORIENT_CMD);
        headingMsb = _readByte();
        headingLsb = _readByte();
        pitch = _readSignedByte();
        roll = _readSignedByte();

        heading = (static_cast<uint16_t>(headingMsb) << 8) | headingLsb;
    }

    float decimal = static_cast<float>(heading % 10) / 10;

    std::vector<float> orientation{
        static_cast<float>(heading / 10) + decimal,
        static_cast<float>(pitch),
        static_cast<float>(roll)};

    return orientation;
}