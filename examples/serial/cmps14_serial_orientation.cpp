/**
 * cmps14_serial_orientation.cpp
 * Author: Ethan Garnier
 * Initialize CMPS14 IMU
 * for serial communication and
 * grab orientation vector.
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
    while (calStatus[3] + calStatus[1] + calStatus[0] < 9)
    {
        calStatus = imu->getCalibrationStatus();
        std::cout << "\nIMU Calibration State: " << std::endl
                  << "\tSystem: " << calStatus[3] << std::endl
                  << "\tGyroscope: " << calStatus[2] << std::endl
                  << "\tAccelerometer: " << calStatus[1] << std::endl
                  << "\tMagnotometer: " << calStatus[0] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    
    while (1)
    {
        std::vector<float> orientation = imu->getOrientation();
        std::cout << "\nHeading: " << orientation[0] << std::endl;
        std::cout << "Pitch: " << orientation[1] << std::endl;
        std::cout << "Roll: " << orientation[2] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 1;
}