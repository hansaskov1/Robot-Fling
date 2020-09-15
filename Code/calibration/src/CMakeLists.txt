cmake_minimum_required(VERSION 3.5)
project(ur_rtde_cmake_example)

find_package(ur_rtde REQUIRED)
add_executable(ur_rtde_cmake_example calibration.cpp)
target_link_libraries(ur_rtde_cmake_example PRIVATE ur_rtde::rtde)
