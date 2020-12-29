cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall -Wno-dev")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.8 REQUIRED)
if (PCL_FOUND)
    message("Gefunden")
    #add_definitions(-DPCL_NO_PRECOMPILE)
else()
    message("Nicht gefunden")
endif ()


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
add_subdirectory(src/quiz/ransac)
add_subdirectory(src/quiz/cluster)

add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp
        src/quiz/ransac/ransac.h src/quiz/ransac/ransac.cpp src/quiz/cluster/cluster.cpp src/quiz/cluster/kdtree.h)
target_link_libraries (environment ${PCL_LIBRARIES})




