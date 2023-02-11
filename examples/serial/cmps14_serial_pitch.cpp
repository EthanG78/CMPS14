/**
 * cmps14_serial_pitch.cpp
 * Author: Ethan Garnier
 * Initialize CMPS14 IMU
 * for serial communication and
 * output pitch information.
 */
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>

#include "cmps14.hpp"

int main()
{
    auto imu = new cmps14(false);

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

    while (1)
    {
        std::cout << "Pitch: " << imu->getPitch() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 1;
}