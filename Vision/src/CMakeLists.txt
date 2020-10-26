cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(Vision)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#find Opencv
find_package(OpenCV REQUIRED)


#find pylon
list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} )
find_package(Pylon REQUIRED)
include_directories(${Pylon_INCLUDE_DIRS})

add_executable(Vision main.cpp calibration.cpp calibration.h objectdetection.cpp objectdetection.h)

target_link_libraries(Vision ${OpenCV_LIBS} ${Pylon_LIBRARIES})
target_link_libraries(Vision pthread)
