cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall -Wno-dev")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
#set(CMAKE_INCLUDE_PATH /usr/local)
#include_directories(/usr/local)

project(playback2)
#find_package(Boost 1.72 REQUIRED)
find_package(PCL 1.7.2 REQUIRED)
if (PCL_FOUND)
    message("blabla")
    #add_definitions(-DPCL_NO_PRECOMPILE)
endif ()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (quizCluster kdtree.cpp ../../render/render.cpp ../ransac/ransac.h ../ransac/ransac.cpp ../../processPointClouds.cpp)
target_link_libraries (quizCluster ${PCL_LIBRARIES})




