cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)


add_subdirectory(pose)

project(calibration)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package (Eigen3 3.3 REQUIRED NO_MODULE)


add_executable(calibration main.cpp)
target_link_libraries(calibration ${PCL_LIBRARIES} Eigen3::Eigen pose)


