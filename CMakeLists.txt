
cmake_minimum_required(VERSION 2.8)
 
PROJECT(SampleBasedReconstruction)
 
find_package(PCL 1.2 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(SBR main.cpp )
 
target_link_libraries(SBR ${PCL_LIBRARIES})
