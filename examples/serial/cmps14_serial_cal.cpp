/**
 * cmps14_serial_cal.cpp
 * Author: Ethan Garnier
 * Initialize CMPS14 IMU
 * for I2C communication and
 * manually calibrate the device
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

    // Enable manual calibration
    imu->toggleBackgroundCal(true);

    // Loop until all sensors are fully calibrated
    while (calStatus[3] + calStatus[2] + calStatus[1] + calStatus[0] < 12)
    {
        calStatus = imu->getCalibrationStatus();
        std::cout << "\nIMU Calibration State: " << std::endl
                  << "\tSystem: " << calStatus[3] << std::endl
                  << "\tGyroscope: " << calStatus[2] << std::endl
                  << "\tAccelerometer: " << calStatus[1] << std::endl
                  << "\tMagnotometer: " << calStatus[0] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    imu->storeCalProfile();

    while (1)
    {
        std::cout << "Heading: " << imu->getHeading() << std::endl;
        std::cout << "Pitch: " << imu->getPitch() << std::endl;
        std::cout << "Roll: " << imu->getRoll() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 1;
}