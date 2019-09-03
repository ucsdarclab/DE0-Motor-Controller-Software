#cmake_minimum_required(VERSION 3.14)
#project(trying)

#set(CMAKE_CXX_STANDARD 14)

#add_executable(trying main.cpp)

cmake_minimum_required(VERSION 3.10)
project(trying)

#set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

SET(ALT_DEVICE_FAMILY soc_cv_av)
add_definitions(-D${ALT_DEVICE_FAMILY})

include_directories("./eigen")

add_executable(trying main.cpp I2CBus.h I2CBus.cpp I2CDevice.h I2CDevice.cpp AS5048b.h AS5048b.cpp fpgaAddressComm.h fpgaEncoder.h motorController.h motorController.cpp main.h heartbeat.cpp)

target_link_libraries(trying "/usr/lib/librobotcontrol.so")
