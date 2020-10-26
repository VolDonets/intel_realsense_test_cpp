cmake_minimum_required(VERSION 3.1.0)
project(intel_realsense_test_cpp)

set(CMAKE_CXX_STANDARD 20)

add_executable(intel_realsense_test_cpp main.cpp)
target_link_libraries(intel_realsense_test_cpp realsense2)

add_executable(rs-capture rs-capture-main.cpp)
target_link_libraries(rs-capture realsense2)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(rs-capture ${OpenCV_LIBS})

add_executable(rs-opencv-depth rs-opencv-depth.cpp)
target_link_libraries(rs-opencv-depth realsense2)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(rs-opencv-depth ${OpenCV_LIBS})