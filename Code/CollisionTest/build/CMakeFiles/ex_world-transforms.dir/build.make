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
CMAKE_SOURCE_DIR = /home/hans/Desktop/RobWorkTest

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hans/Desktop/RobWorkTest/build

# Include any dependencies generated for this target.
include CMakeFiles/ex_world-transforms.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ex_world-transforms.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ex_world-transforms.dir/flags.make

CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.o: CMakeFiles/ex_world-transforms.dir/flags.make
CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.o: ../ex_world-transforms.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hans/Desktop/RobWorkTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.o -c /home/hans/Desktop/RobWorkTest/ex_world-transforms.cpp

CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hans/Desktop/RobWorkTest/ex_world-transforms.cpp > CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.i

CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hans/Desktop/RobWorkTest/ex_world-transforms.cpp -o CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.s

CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.o.requires:

.PHONY : CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.o.requires

CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.o.provides: CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.o.requires
	$(MAKE) -f CMakeFiles/ex_world-transforms.dir/build.make CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.o.provides.build
.PHONY : CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.o.provides

CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.o.provides.build: CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.o


# Object files for target ex_world-transforms
ex_world__transforms_OBJECTS = \
"CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.o"

# External object files for target ex_world-transforms
ex_world__transforms_EXTERNAL_OBJECTS =

libex_world-transforms.a: CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.o
libex_world-transforms.a: CMakeFiles/ex_world-transforms.dir/build.make
libex_world-transforms.a: CMakeFiles/ex_world-transforms.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hans/Desktop/RobWorkTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libex_world-transforms.a"
	$(CMAKE_COMMAND) -P CMakeFiles/ex_world-transforms.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ex_world-transforms.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ex_world-transforms.dir/build: libex_world-transforms.a

.PHONY : CMakeFiles/ex_world-transforms.dir/build

CMakeFiles/ex_world-transforms.dir/requires: CMakeFiles/ex_world-transforms.dir/ex_world-transforms.cpp.o.requires

.PHONY : CMakeFiles/ex_world-transforms.dir/requires

CMakeFiles/ex_world-transforms.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ex_world-transforms.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ex_world-transforms.dir/clean

CMakeFiles/ex_world-transforms.dir/depend:
	cd /home/hans/Desktop/RobWorkTest/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hans/Desktop/RobWorkTest /home/hans/Desktop/RobWorkTest /home/hans/Desktop/RobWorkTest/build /home/hans/Desktop/RobWorkTest/build /home/hans/Desktop/RobWorkTest/build/CMakeFiles/ex_world-transforms.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ex_world-transforms.dir/depend

