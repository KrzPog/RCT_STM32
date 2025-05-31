cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wextra")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wconversion")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wsign-conversion")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wsign-compare")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pedantic")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wconversion")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wsign-conversion")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wsign-compare")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic")

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME RCT_STM32)
set(EXECUTABLE_NAME "RCT_STM32")

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${EXECUTABLE_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

add_subdirectory(Middlewares/FreeMODBUS)

# Link directories setup
target_link_directories(${EXECUTABLE_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${EXECUTABLE_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_CURRENT_LIST_DIR}/Src/App/dataFlash.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/App/turretStates.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/App/pid_cascade.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/App/pid_output.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/App/pid_position.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/App/pid_speed.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/App/speedControl.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/ModbusRegisters/reg_holding.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/ModbusRegisters/reg_input.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/Sensors/encoders.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/Sensors/analog_sensors.c
    ${CMAKE_CURRENT_LIST_DIR}/Src/Sensors/accelerometer.c

)

# Add include paths
target_include_directories(${EXECUTABLE_NAME} PRIVATE
    # Add user defined include paths
    Middlewares/FreeMODBUS
)

# Add project symbols (macros)
target_compile_definitions(${EXECUTABLE_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${EXECUTABLE_NAME}
    stm32cubemx

    # Add user defined libraries
    FreeMODBUS
)
