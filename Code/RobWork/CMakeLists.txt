# Test CMake version
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# The name of the project. (EDIT THIS)
PROJECT(TutorialCollection)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

##### IMPORTANT EDIT THESE PATHS TO REFLECT YOUR ROBWORK INSTALLATION #####
# Now set the RW/RWS root (edit this if necessary)
#
# Set the RobWork root (edit in .bashrc if necessary)
SET(RW_ROOT C:\Users\chris\Documents\QT projekter\Robot-Fling\Code\RobWork)                      # Use the environment variable  $RW_ROOT from .bashrc file (NOT necessary in PPA install)

# set some robwork settings (EDIT THESE TO FIT YOUR ENVIRONMENT)2

# Set build type to release
SET(CMAKE_BUILD_TYPE Release)
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

# Include default settings for constructing a robwork dependent project
SET(RobWork_DIR ${RW_ROOT}/cmake)                # (NOT necessary in PPA install)
FIND_PACKAGE(RobWork REQUIRED)
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# if you have additional libraries or include dirs then add them here
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )

# And now we add any targets that we want
add_executable(HelloWorld src/HelloWorld.cpp)
target_link_libraries(HelloWorld ${ROBWORK_LIBRARIES})

# add another executable
add_executable(MathJoggling src/MathJoggling.cpp)
target_link_libraries(MathJoggling ${ROBWORK_LIBRARIES})
