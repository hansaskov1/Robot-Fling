# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hans/GithubCode/Robot-Fling/Code/CollisionTest

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hans/GithubCode/Robot-Fling/Code/build-CollisionTest-Desktop-Default

# Include any dependencies generated for this target.
include CMakeFiles/ex-collisions2.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ex-collisions2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ex-collisions2.dir/flags.make

CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.o: CMakeFiles/ex-collisions2.dir/flags.make
CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.o: /home/hans/GithubCode/Robot-Fling/Code/CollisionTest/ex-collisions2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hans/GithubCode/Robot-Fling/Code/build-CollisionTest-Desktop-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.o"
	/usr/bin/g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.o -c /home/hans/GithubCode/Robot-Fling/Code/CollisionTest/ex-collisions2.cpp

CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.i"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hans/GithubCode/Robot-Fling/Code/CollisionTest/ex-collisions2.cpp > CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.i

CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.s"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hans/GithubCode/Robot-Fling/Code/CollisionTest/ex-collisions2.cpp -o CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.s

CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.o.requires:

.PHONY : CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.o.requires

CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.o.provides: CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.o.requires
	$(MAKE) -f CMakeFiles/ex-collisions2.dir/build.make CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.o.provides.build
.PHONY : CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.o.provides

CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.o.provides.build: CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.o


# Object files for target ex-collisions2
ex__collisions2_OBJECTS = \
"CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.o"

# External object files for target ex-collisions2
ex__collisions2_EXTERNAL_OBJECTS =

ex-collisions2: CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.o
ex-collisions2: CMakeFiles/ex-collisions2.dir/build.make
ex-collisions2: /usr/lib/x86_64-linux-gnu/libyaobi.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libpqp.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/RobWork/static/libsdurw_qhull.a
ex-collisions2: /usr/lib/x86_64-linux-gnu/RobWork/static/libsdurw_csgjs.a
ex-collisions2: /usr/lib/x86_64-linux-gnu/libsdurw_algorithms.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libsdurw_pathplanners.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libsdurw_pathoptimization.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libsdurw_simulation.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libsdurw_opengl.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libsdurw_assembly.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libsdurw_task.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libsdurw_calibration.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libsdurw_csg.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libsdurw_control.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libsdurw_proximitystrategies.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libsdurw.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libsdurw_core.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libsdurw_common.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libsdurw_math.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libfcl.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/liblua5.3.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libm.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libassimp.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libGL.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libGLU.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libxerces-c.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libdl.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libboost_regex.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libboost_system.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libboost_thread.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libpthread.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libboost_test_exec_monitor.a
ex-collisions2: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/librtde.so.1.2.6
ex-collisions2: /usr/lib/x86_64-linux-gnu/libboost_system.so
ex-collisions2: /usr/lib/x86_64-linux-gnu/libboost_thread.so
ex-collisions2: CMakeFiles/ex-collisions2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hans/GithubCode/Robot-Fling/Code/build-CollisionTest-Desktop-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ex-collisions2"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ex-collisions2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ex-collisions2.dir/build: ex-collisions2

.PHONY : CMakeFiles/ex-collisions2.dir/build

CMakeFiles/ex-collisions2.dir/requires: CMakeFiles/ex-collisions2.dir/ex-collisions2.cpp.o.requires

.PHONY : CMakeFiles/ex-collisions2.dir/requires

CMakeFiles/ex-collisions2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ex-collisions2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ex-collisions2.dir/clean

CMakeFiles/ex-collisions2.dir/depend:
	cd /home/hans/GithubCode/Robot-Fling/Code/build-CollisionTest-Desktop-Default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hans/GithubCode/Robot-Fling/Code/CollisionTest /home/hans/GithubCode/Robot-Fling/Code/CollisionTest /home/hans/GithubCode/Robot-Fling/Code/build-CollisionTest-Desktop-Default /home/hans/GithubCode/Robot-Fling/Code/build-CollisionTest-Desktop-Default /home/hans/GithubCode/Robot-Fling/Code/build-CollisionTest-Desktop-Default/CMakeFiles/ex-collisions2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ex-collisions2.dir/depend

