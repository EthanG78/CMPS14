/**
 * cmps14_i2c_heading.cpp
 * Author: Ethan Garnier
 * Initialize CMPS14 IMU
 * for I2C communication and
 * output heading information.
*/
#include <iostream>
#include <chrono>
#include <thread>

#include "cmps14.hpp"

int main()
{
    auto imu = new cmps14(true);

    if (imu->begin() == -1)
    {
        return 0;
    }

    std::cout << "IMU Software Version: " << imu->getSoftwareVersion() << std::endl;

    while (1)
    {
        std::cout << "Heading: " << imu->getHeading() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 1;
}