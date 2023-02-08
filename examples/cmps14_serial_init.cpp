/**
 * cmps14_serial_init.cpp
 * Author: Ethan Garnier
 * A simple example programming
 * showing how to initialize
 * CMPS14 communication over
 * SERIAL using the default address.
*/
#include <iostream>
#include "cmps14.hpp"

int main()
{
    // By passing false, we are specifying that we
    // want to communicate over serial.
    // If we wanted to specify the serial port, we would
    // instead use: new cmps14(serialPort);
    auto imu = new cmps14(false);

    if (imu->begin() == -1)
    {
        return 0;
    }

    std::cout << "IMU Software Version: " << imu->getSoftwareVersion() << std::endl;
    std::cout << "IMU Calibration State: " << imu->getCalibrationStatus() << std::endl;

    return 1;
}