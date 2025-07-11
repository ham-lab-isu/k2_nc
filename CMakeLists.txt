cmake_minimum_required(VERSION 3.10)
project(kawasaki_gcode_driver)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

file(GLOB SOURCES "src/*.cpp")
add_executable(gcode_driver ${SOURCES})

