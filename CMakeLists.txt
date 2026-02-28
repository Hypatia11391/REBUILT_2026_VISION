cmake_minimum_required(VERSION 3.16)
project(FRCVision)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# 1. Find Dependencies
find_package(libcamera REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# AprilTag usually doesn't have a standard CMake config, so we find it manually
find_path(APRILTAG_INCLUDE_DIR NAMES apriltag/apriltag.h)
find_library(APRILTAG_LIBRARY NAMES apriltag)

# 2. Define the Executable
add_executable(vision_system main.cpp)

# 3. Tell the compiler where the "Headers" are
target_include_directories(vision_system PRIVATE 
    ${libcamera_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${APRILTAG_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR} # This is for your "header.h"
)

# 4. Tell the compiler which "Libraries" to link
target_link_libraries(vision_system PRIVATE 
    libcamera::libcamera
    ${OpenCV_LIBS}
    ${APRILTAG_LIBRARY}
)