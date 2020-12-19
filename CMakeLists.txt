cmake_minimum_required(VERSION 3.10)
project(SensorFusion)

set(CMAKE_CXX_STANDARD 14)
include_directories(SFND_Lidar_Obstacle_Detection)
add_subdirectory(SFND_Lidar_Obstacle_Detection)


add_executable(SensorFusion main.cpp)
