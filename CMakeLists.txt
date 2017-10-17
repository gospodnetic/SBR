
cmake_minimum_required(VERSION 2.8)
 
PROJECT(SampleBasedReconstruction)
 
find_package(PCL 1.2 REQUIRED)

set (CMAKE_CXX_FLAGS "-std=gnu++11 ${CMAKE_CXX_FLAGS}")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Get LodePNG source files
SET (LODEPNG_FILES 
    ${PROJECT_SOURCE_DIR}/lodepng/lodepng.cpp
    ${PROJECT_SOURCE_DIR}/lodepng/lodepng_util)

# Get all source *.cpp files recursively
file(GLOB_RECURSE SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_executable(SBR ${SRC_FILES} ${LODEPNG_FILES})
 
target_link_libraries(SBR ${PCL_LIBRARIES})
