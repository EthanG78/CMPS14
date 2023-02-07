/**
 * cmps14.hpp
 * Author: Ethan Garnier
 */
#pragma once

class cmps14
{
private:
    bool _i2c = false;

public:
    cmps14(bool i2c);
    ~cmps14();

    int begin();
};