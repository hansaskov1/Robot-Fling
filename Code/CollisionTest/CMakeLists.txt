cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(ExampleSnippets)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "None")
endif()
set(RW_ROOT "$ENV{RW_ROOT}")
#set(RobWork_DIR "../../cmake")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")
find_package(ur_rtde REQUIRED)

link_directories(${ROBWORK_LIBRARY_DIRS})

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


rw_build(ex-collisions2)
target_link_libraries(ex-collisions2 ur_rtde::rtde)






foreach(obj ${ROBWORK_INCLUDE_DIRS})
    message(STATUS "INCL_DIR: ${obj}")
endforeach()
foreach(obj ${ROBWORK_LIBRARY_DIRS})
    message(STATUS "LIB_DIR: ${obj}")
endforeach()
foreach(obj ${ROBWORK_LIBRARIES})
    message(STATUS "LIB: ${obj}")
endforeach()
