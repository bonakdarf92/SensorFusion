cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall -Wno-dev")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.8 REQUIRED)
if (PCL_FOUND)
    message("blabla")
    #add_definitions(-DPCL_NO_PRECOMPILE)
endif ()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (quizCluster cluster.cpp ../../render/render.cpp ../ransac/ransac.h ../ransac/ransac.cpp ../../processPointClouds.cpp)
target_link_libraries (quizCluster ${PCL_LIBRARIES})




