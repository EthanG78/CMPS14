/**
 * cmps14_init.cpp
 * Author: Ethan Garnier
 * A simple example programming
 * showing how to initialize
 * CMPS14 communication over
 * I2C using the default address.
*/
#include <iostream>
#include "cmps14.hpp"

int main()
{
    auto imu = new cmps14(true);

    if (imu->begin() == -1)
    {
        return 0;
    }

    std::cout << "IMU Software Version: " << imu->getSoftwareVersion() << std::endl;

    return 1;
}