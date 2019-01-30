cmake_minimum_required(VERSION 3.0)

project(calibration)


find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package (Eigen3 3.3 REQUIRED NO_MODULE)



INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})

SET(calibration_HEADERS
                pose.h
                camera.h
                lidar.h)

SET(calibration_SOURCES
              src/pose.cpp
              src/camera.cpp
              src/lidar.cpp)

add_executable(calibration main.cpp ${calibration_HEADERS} ${calibration_SOURCES})
target_link_libraries(calibration ${PCL_LIBRARIES} Eigen3::Eigen)


