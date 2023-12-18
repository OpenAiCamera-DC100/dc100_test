cmake_minimum_required(VERSION 3.10)
project(stereo_calib)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(stereo_calib stereo_calib.cpp)
target_link_libraries(stereo_calib ${OpenCV_LIBS})

add_executable(stereo_match stereo_match.cpp)
target_link_libraries(stereo_match ${OpenCV_LIBS})