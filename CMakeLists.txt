cmake_minimum_required(VERSION 3.16)
project(FRCVision)

# Use C++17 for Eigen and modern libcamera support
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Find Dependencies
# OpenCV and Eigen usually have standard configs
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# libcamera often uses PkgConfig on Raspberry Pi
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera libcamera-base)

# AprilTag manual find (since it rarely has a CMake config)
find_path(APRILTAG_INCLUDE_DIR NAMES apriltag/apriltag.h)
find_library(APRILTAG_LIBRARY NAMES apriltag)

# 2. Define the Executable (ONLY ONCE)
add_executable(vision_system main.cpp)

# 3. Tell the compiler where the "Headers" are
# This tells the compiler where to find the #include files
target_include_directories(vision_system PRIVATE 
    ${OpenCV_INCLUDE_DIRS}
    ${LIBCAMERA_INCLUDE_DIRS}
    ${APRILTAG_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR} 
)

# 4. Tell the compiler which "Libraries" to link
# This connects your code to the actual compiled logic of these libraries
target_link_libraries(vision_system PRIVATE 
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
    ${APRILTAG_LIBRARY}
    Eigen3::Eigen
)