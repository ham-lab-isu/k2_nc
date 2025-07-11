cmake_minimum_required(VERSION 3.10)
project(k2_nc)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies if they are needed
find_package(ament_cmake REQUIRED)

add_executable(main src/main.cpp)

# target the dependencies if needed

# add compiler features
target_compile_features(main PUBLIC cxx_std_17)

# add the include directories
target_include_directories(main PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

## link the KRNX library
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

# link the krnx library to the main executable
target_link_libraries(main ${krnx_LIBRARIES} ${krnx2_LIBRARIES})

# install the directories and executables
install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)
install(
    DIRECTORY gcode/
    DESTINATION share/${PROJECT_NAME}
)
install(
    FILES ${krnx_LIBRARIES} ${krnx2_LIBRARIES}
    DESTINATION lib/
)
install(
    TARGETS main
    RUNTIME DESTINATION bin
)

ament_export_targets(main HAS_LIBRARY_TARGET)
ament_package()
