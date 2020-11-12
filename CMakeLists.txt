cmake_minimum_required(VERSION 3.17)
project(hand_calibration)

find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(hand_calibration main.cpp)
target_link_libraries (hand_calibration ${OpenCV_LIBS} ${PCL_LIBRARIES})

