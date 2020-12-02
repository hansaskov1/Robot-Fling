cmake_minimum_required(VERSION 3.5)

project(SemesterProjekt LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(RW_ROOT "$ENV{RW_ROOT}")

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check http://doc.qt.io/qt-5/deployment-android.html for more information.
# They need to be set before the find_package(Qt5 ...) call.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Sql)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Network)
find_package(OpenCV REQUIRED)
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")
find_package(ur_rtde REQUIRED)
list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} )
find_package(Pylon REQUIRED)
link_directories(${ROBWORK_LIBRARY_DIRS})
include_directories(${Pylon_INCLUDE_DIRS})

include_directories("${PROJECT_SOURCE_DIR}/clientInfo.h")

macro(rw_build _name)
    # And now we add any targets that we want
    add_executable(${_name} ${_name}.cpp)
    target_link_libraries(${_name} ${ROBWORK_LIBRARIES})
    target_include_directories(${_name} PRIVATE ${ROBWORK_INCLUDE_DIRS})
endmacro()

macro(rw_build_lib _name)
    # And now we add any targets that we want
    add_library(${_name} ${_name}.cpp)
    target_link_libraries(${_name} ${ROBWORK_LIBRARIES})
    target_include_directories(${_name} PRIVATE ${ROBWORK_INCLUDE_DIRS})
endmacro()

if(ANDROID)
  add_library(SemesterProjekt SHARED
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    gripper.h
    sqlcontroller.h
    sqlcontroller.cpp
    Throw.h
    calibration.h
    calibration.cpp
    objectdetection.h
    objectdetection.cpp
    RobotControl.h
    DetectCollision.h
    path.h
  )
else()
  add_executable(SemesterProjekt
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    Gripper.h
    sqlcontroller.h
    sqlcontroller.cpp
    Throw.h
    calibration.h
    calibration.cpp
    objectdetection.h
    objectdetection.cpp
    RobotControl.h
    DetectCollision.h
    path.h
  )
endif()
target_link_libraries(SemesterProjekt ${ROBWORK_LIBRARIES})
target_include_directories(SemesterProjekt PRIVATE ${ROBWORK_INCLUDE_DIRS})
target_link_libraries(SemesterProjekt ur_rtde::rtde)
target_link_libraries(SemesterProjekt Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::Sql Qt${QT_VERSION_MAJOR}::Network ${OpenCV_LIBS} ${Pylon_LIBRARIES} pthread)

foreach(obj ${ROBWORK_INCLUDE_DIRS})
    message(STATUS "INCL_DIR: ${obj}")
endforeach()
foreach(obj ${ROBWORK_LIBRARY_DIRS})
    message(STATUS "LIB_DIR: ${obj}")
endforeach()
foreach(obj ${ROBWORK_LIBRARIES})
    message(STATUS "LIB: ${obj}")
endforeach()
