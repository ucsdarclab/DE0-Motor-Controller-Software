#cmake_minimum_required(VERSION 3.14)
#project(trying)

#set(CMAKE_CXX_STANDARD 14)

#add_executable(trying main.cpp)

cmake_minimum_required(VERSION 3.10)
project(DE0_Motor_Controller)

#set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

SET(ALT_DEVICE_FAMILY soc_cv_av)
add_definitions(-D${ALT_DEVICE_FAMILY})


include_directories( include
        "./eigen"
        ini)

add_library(ini_library
        ini/ini.c
        ini/ini.h
        ini/INIReader.cpp
        ini/INIReader.h
)

add_library(${PROJECT_NAME} STATIC
        include/${PROJECT_NAME}/AS5048b.h
        include/${PROJECT_NAME}/fpgaAddressComm.h
        include/${PROJECT_NAME}/fpgaEncoder.h
        include/${PROJECT_NAME}/I2CBus.h
        include/${PROJECT_NAME}/I2CDevice.h
        include/${PROJECT_NAME}/motorController.h
        include/fpga_headers/alt_gpio.h
        include/fpga_headers/hps.h
        include/fpga_headers/hps_0.h
        include/fpga_headers/hwlib.h
        include/fpga_headers/socal.h
        src/AS5048b.cpp
        src/I2CBus.cpp
        src/I2CDevice.cpp
        src/motorController.cpp
)

#Allows other projects that use this cmake as a subdirectory to access all the includes!!!!

set(DE0-Motor-Controller_INCLUDE_DIRS
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/eigen
        ${PROJECT_SOURCE_DIR}/ini
        CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)


set_target_properties(${PROJECT_NAME}
        PROPERTIES POSITION_INDEPENDENT_CODE ON)

#Building the test executable
add_executable(test main/main.cpp main/main.h)
target_link_libraries(test "/usr/lib/librobotcontrol.so"
            ini_library
            ${PROJECT_NAME})
