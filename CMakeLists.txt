
cmake_minimum_required(VERSION 2.8)
 
PROJECT(SampleBasedReconstruction)
 
set (CMAKE_CXX_FLAGS "-std=gnu++11 ${CMAKE_CXX_FLAGS}")

find_package(PCL 1.2 REQUIRED)
find_package(YAML-CPP)

include_directories(${PCL_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Get LodePNG source files
SET (LODEPNG_FILES 
    ${PROJECT_SOURCE_DIR}/submodules/lodepng/lodepng.cpp
    ${PROJECT_SOURCE_DIR}/submodules/lodepng/lodepng_util.cpp)

# Get all source *.cpp files recursively
file(GLOB_RECURSE SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_executable(SBR ${SRC_FILES} ${LODEPNG_FILES})
target_link_libraries(SBR ${PCL_LIBRARIES} cnpy z ${YAML_CPP_LIBRARIES})
