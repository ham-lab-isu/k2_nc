cmake_minimum_required(VERSION 3.8)
project(k2_nc)

# Export compile commands for VS Code
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler-specific options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${PROJECT_NAME}/include
)

## KRNX
message(STATUS "CMAKE_SYSTEM_NAME : ${CMAKE_SYSTEM_NAME}")
message(STATUS "CMAKE_SYSTEM_PROCESSOR : ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "CMAKE_HOST_SYSTEM_NAME : ${CMAKE_HOST_SYSTEM_NAME}")
message(STATUS "CMAKE_HOST_SYSTEM_PROCESSOR : ${CMAKE_HOST_SYSTEM_PROCESSOR}")
message(STATUS "CMAKE_LIBRARY_ARCHITECTURE : ${CMAKE_LIBRARY_ARCHITECTURE}")
if( ${CMAKE_LIBRARY_ARCHITECTURE} MATCHES ".*x86_64.*" )
  set(krnx_arch x86_64)
  message("Arch set as .*x86_64.*")
elseif( ${CMAKE_LIBRARY_ARCHITECTURE} MATCHES ".*aarch64.*" )
  set(krnx_arch aarch64)
  message("Arch set as aarch64")
elseif( ${CMAKE_LIBRARY_ARCHITECTURE} MATCHES ".*i386.*" )
  set(krnx_arch i386)
  message("Arch set as i386")
elseif( ${CMAKE_LIBRARY_ARCHITECTURE} MATCHES ".*arm.*gnueabihf" )
  set(krnx_arch armhf)
  message("Arch set as armhf")
elseif( ${CMAKE_LIBRARY_ARCHITECTURE} MATCHES ".*arm.*gnueabi" )
  set(krnx_arch armel)
  message("Arch set as armel")
else()
  message(WARNING "\nCould not detect system processor architecture: assuming 'x86_64'. This may result in non-working binaries.")
  set(krnx_arch x86_64)
endif()

# This defines the library variable krnx_LIBRARIES which can then be called as ${krnx_LIBRARIES}
find_library(krnx_LIBRARIES libkrnx.so PATHS ${PROJECT_SOURCE_DIR}/lib/${krnx_arch})
find_library(krnx2_LIBRARIES libkrnx.so.2 PATHS ${PROJECT_SOURCE_DIR}/lib/${krnx_arch})

message("Found KRNX library at: ${krnx_LIBRARIES}")
message("Found KRNX2 library at: ${krnx2_LIBRARIES}")


# Collect source files
file(GLOB SOURCES "src/*.cpp")

# Create executable
add_executable(gcode_driver ${SOURCES})

# Dependencies
ament_target_dependencies(gcode_driver
  rclcpp
  std_msgs
)

# Link libraries
target_link_libraries(gcode_driver
  ${krnx_LIBRARIES}
  ${krnx2_LIBRARIES}
)

# Install targets
install(TARGETS
  gcode_driver
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()