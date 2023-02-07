# CMPS14
A C++ driver for the [CMPS14 Tilt Compensated Magnetic Compass](https://ca.robotshop.com/products/tilt-compensated-magnetic-compass-cmps14). 

# WIP
As of February 2023, this is a work in progress.

# How to build
The following commands have been tested on CentOS 8.5 and Debian 11 (bullseye)


To build and install this library, first clone the repository:
```console
user@machine ~$ git clone https://github.com/EthanG78/CMPS14.git
```
Enter the repository directory and create a build directory:
```console
user@machine ~$ cd CMPS14/
user@machine CMPS14$ mkdir build
```
Enter the build directory and configure the project using CMake:
```console
user@machine CMPS14$ cd build/
user@machine build$ cmake -S .. -B .
```
Build the project using CMake:
```console
user@machine build$ cmake --build .
```
Finally, install the project to your machine:
```console
user@machine build$ sudo make install
```

Author: Ethan Garnier