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
CMAKE_SOURCE_DIR = /home/suspend/GitHub/Robot-Fling/Vision/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/suspend/GitHub/Robot-Fling/Vision/build-src-Desktop-Default

# Include any dependencies generated for this target.
include CMakeFiles/Vision.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Vision.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Vision.dir/flags.make

CMakeFiles/Vision.dir/main.cpp.o: CMakeFiles/Vision.dir/flags.make
CMakeFiles/Vision.dir/main.cpp.o: /home/suspend/GitHub/Robot-Fling/Vision/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/suspend/GitHub/Robot-Fling/Vision/build-src-Desktop-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Vision.dir/main.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Vision.dir/main.cpp.o -c /home/suspend/GitHub/Robot-Fling/Vision/src/main.cpp

CMakeFiles/Vision.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Vision.dir/main.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/suspend/GitHub/Robot-Fling/Vision/src/main.cpp > CMakeFiles/Vision.dir/main.cpp.i

CMakeFiles/Vision.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Vision.dir/main.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/suspend/GitHub/Robot-Fling/Vision/src/main.cpp -o CMakeFiles/Vision.dir/main.cpp.s

CMakeFiles/Vision.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/Vision.dir/main.cpp.o.requires

CMakeFiles/Vision.dir/main.cpp.o.provides: CMakeFiles/Vision.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/Vision.dir/build.make CMakeFiles/Vision.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/Vision.dir/main.cpp.o.provides

CMakeFiles/Vision.dir/main.cpp.o.provides.build: CMakeFiles/Vision.dir/main.cpp.o


CMakeFiles/Vision.dir/calibration.cpp.o: CMakeFiles/Vision.dir/flags.make
CMakeFiles/Vision.dir/calibration.cpp.o: /home/suspend/GitHub/Robot-Fling/Vision/src/calibration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/suspend/GitHub/Robot-Fling/Vision/build-src-Desktop-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Vision.dir/calibration.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Vision.dir/calibration.cpp.o -c /home/suspend/GitHub/Robot-Fling/Vision/src/calibration.cpp

CMakeFiles/Vision.dir/calibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Vision.dir/calibration.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/suspend/GitHub/Robot-Fling/Vision/src/calibration.cpp > CMakeFiles/Vision.dir/calibration.cpp.i

CMakeFiles/Vision.dir/calibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Vision.dir/calibration.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/suspend/GitHub/Robot-Fling/Vision/src/calibration.cpp -o CMakeFiles/Vision.dir/calibration.cpp.s

CMakeFiles/Vision.dir/calibration.cpp.o.requires:

.PHONY : CMakeFiles/Vision.dir/calibration.cpp.o.requires

CMakeFiles/Vision.dir/calibration.cpp.o.provides: CMakeFiles/Vision.dir/calibration.cpp.o.requires
	$(MAKE) -f CMakeFiles/Vision.dir/build.make CMakeFiles/Vision.dir/calibration.cpp.o.provides.build
.PHONY : CMakeFiles/Vision.dir/calibration.cpp.o.provides

CMakeFiles/Vision.dir/calibration.cpp.o.provides.build: CMakeFiles/Vision.dir/calibration.cpp.o


CMakeFiles/Vision.dir/objectdetection.cpp.o: CMakeFiles/Vision.dir/flags.make
CMakeFiles/Vision.dir/objectdetection.cpp.o: /home/suspend/GitHub/Robot-Fling/Vision/src/objectdetection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/suspend/GitHub/Robot-Fling/Vision/build-src-Desktop-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Vision.dir/objectdetection.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Vision.dir/objectdetection.cpp.o -c /home/suspend/GitHub/Robot-Fling/Vision/src/objectdetection.cpp

CMakeFiles/Vision.dir/objectdetection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Vision.dir/objectdetection.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/suspend/GitHub/Robot-Fling/Vision/src/objectdetection.cpp > CMakeFiles/Vision.dir/objectdetection.cpp.i

CMakeFiles/Vision.dir/objectdetection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Vision.dir/objectdetection.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/suspend/GitHub/Robot-Fling/Vision/src/objectdetection.cpp -o CMakeFiles/Vision.dir/objectdetection.cpp.s

CMakeFiles/Vision.dir/objectdetection.cpp.o.requires:

.PHONY : CMakeFiles/Vision.dir/objectdetection.cpp.o.requires

CMakeFiles/Vision.dir/objectdetection.cpp.o.provides: CMakeFiles/Vision.dir/objectdetection.cpp.o.requires
	$(MAKE) -f CMakeFiles/Vision.dir/build.make CMakeFiles/Vision.dir/objectdetection.cpp.o.provides.build
.PHONY : CMakeFiles/Vision.dir/objectdetection.cpp.o.provides

CMakeFiles/Vision.dir/objectdetection.cpp.o.provides.build: CMakeFiles/Vision.dir/objectdetection.cpp.o


# Object files for target Vision
Vision_OBJECTS = \
"CMakeFiles/Vision.dir/main.cpp.o" \
"CMakeFiles/Vision.dir/calibration.cpp.o" \
"CMakeFiles/Vision.dir/objectdetection.cpp.o"

# External object files for target Vision
Vision_EXTERNAL_OBJECTS =

Vision: CMakeFiles/Vision.dir/main.cpp.o
Vision: CMakeFiles/Vision.dir/calibration.cpp.o
Vision: CMakeFiles/Vision.dir/objectdetection.cpp.o
Vision: CMakeFiles/Vision.dir/build.make
Vision: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
Vision: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
Vision: CMakeFiles/Vision.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/suspend/GitHub/Robot-Fling/Vision/build-src-Desktop-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable Vision"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Vision.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Vision.dir/build: Vision

.PHONY : CMakeFiles/Vision.dir/build

CMakeFiles/Vision.dir/requires: CMakeFiles/Vision.dir/main.cpp.o.requires
CMakeFiles/Vision.dir/requires: CMakeFiles/Vision.dir/calibration.cpp.o.requires
CMakeFiles/Vision.dir/requires: CMakeFiles/Vision.dir/objectdetection.cpp.o.requires

.PHONY : CMakeFiles/Vision.dir/requires

CMakeFiles/Vision.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Vision.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Vision.dir/clean

CMakeFiles/Vision.dir/depend:
	cd /home/suspend/GitHub/Robot-Fling/Vision/build-src-Desktop-Default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/suspend/GitHub/Robot-Fling/Vision/src /home/suspend/GitHub/Robot-Fling/Vision/src /home/suspend/GitHub/Robot-Fling/Vision/build-src-Desktop-Default /home/suspend/GitHub/Robot-Fling/Vision/build-src-Desktop-Default /home/suspend/GitHub/Robot-Fling/Vision/build-src-Desktop-Default/CMakeFiles/Vision.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Vision.dir/depend

