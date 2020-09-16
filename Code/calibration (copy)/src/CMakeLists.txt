cmake_minimum_required(VERSION 3.5)
project(ur_rtde_cmake_example)

find_package(ur_rtde REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${/home/hans/eigen-3.3.7})
add_executable(ur_rtde_cmake_example calibration.cpp)
target_link_libraries(ur_rtde_cmake_example PRIVATE ur_rtde::rtde)
