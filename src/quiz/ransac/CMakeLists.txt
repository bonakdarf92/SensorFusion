cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall -Wno-dev")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.8 REQUIRED)
if (PCL_FOUND)
    message("Blabla")
    #add_definitions(-DPCL_NO_PRECOMPILE)
else()
    message("Nicht gefunden")
endif ()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (quizRansac ransac.h ../../render/render.cpp ../../render/render.h ransac2d.cpp)
target_link_libraries (quizRansac ${PCL_LIBRARIES})




