cmake_minimum_required(VERSION 3.7...3.25)

if(${CMAKE_VERSION} VERSION_LESS 3.12)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()

project(CMPS14 VERSION 1.0
                  DESCRIPTION "C++ Driver for CMPS14 Tilt Compensated Magnetic Compass"
                  LANGUAGES CXX)

add_library(libcmps14 STATIC src/cmps14.cpp include/cmps14/cmps14.hpp)
set_target_properties(libcmps14 PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(libcmps14 PROPERTIES PUBLIC_HEADER include/cmps14/cmps14.hpp)
target_include_directories(libcmps14 PUBLIC include)

include(GNUInstallDirs)
install(TARGETS libcmps14
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})