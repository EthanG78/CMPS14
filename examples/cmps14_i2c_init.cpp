/**
 * cmps14_i2c_init.cpp
 * Author: Ethan Garnier
 * A simple example programming
 * showing how to initialize
 * CMPS14 communication over
 * I2C using the default address.
 */
#include <iostream>
#include <vector>

#include "cmps14.hpp"

int main()
{
    auto imu = new cmps14(true);

    if (imu->begin() == -1)
    {
        return 0;
    }

    std::cout << "IMU Software Version: " << imu->getSoftwareVersion() << std::endl;
    std::vector<int> calStatus = imu->getCalibrationStatus();
    std::cout << "IMU Calibration State: " << std::endl
              << "\tSystem: " << calStatus[3] << std::endl
              << "\tGyroscope: " << calStatus[2] << std::endl
              << "\tAccelerometer: " << calStatus[1] << std::endl
              << "\tMagnotometer: " << calStatus[0] << std::endl;

    return 1;
}