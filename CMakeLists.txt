cmake_minimum_required(VERSION 3.7...3.25)

if(${CMAKE_VERSION} VERSION_LESS 3.12)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()

project(
    CMPS14 
    VERSION 1.0
    DESCRIPTION "C++ Driver for CMPS14 Tilt Compensated Magnetic Compass"
    LANGUAGES CXX
)

add_subdirectory(examples)

add_library(
    cmps14  
    src/cmps14.cpp include/cmps14/cmps14.hpp
)

set_target_properties(cmps14 PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(cmps14 PROPERTIES PUBLIC_HEADER include/cmps14/cmps14.hpp)
target_include_directories(cmps14 PUBLIC include)

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
target_link_libraries(cmps14 ${WIRINGPI_LIBRARIES})

include(GNUInstallDirs)
install(
    TARGETS cmps14
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)