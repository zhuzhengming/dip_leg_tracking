# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/zhuzhengming/software/clion-2021.2.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/zhuzhengming/software/clion-2021.2.3/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zhuzhengming/dip_ws/src/image_pkg

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zhuzhengming/dip_ws/src/image_pkg/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/colormove.dir/depend.make
# Include the progress variables for this target.
include CMakeFiles/colormove.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/colormove.dir/flags.make

CMakeFiles/colormove.dir/src/colormove.cpp.o: CMakeFiles/colormove.dir/flags.make
CMakeFiles/colormove.dir/src/colormove.cpp.o: ../src/colormove.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhuzhengming/dip_ws/src/image_pkg/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/colormove.dir/src/colormove.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/colormove.dir/src/colormove.cpp.o -c /home/zhuzhengming/dip_ws/src/image_pkg/src/colormove.cpp

CMakeFiles/colormove.dir/src/colormove.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/colormove.dir/src/colormove.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhuzhengming/dip_ws/src/image_pkg/src/colormove.cpp > CMakeFiles/colormove.dir/src/colormove.cpp.i

CMakeFiles/colormove.dir/src/colormove.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/colormove.dir/src/colormove.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhuzhengming/dip_ws/src/image_pkg/src/colormove.cpp -o CMakeFiles/colormove.dir/src/colormove.cpp.s

# Object files for target colormove
colormove_OBJECTS = \
"CMakeFiles/colormove.dir/src/colormove.cpp.o"

# External object files for target colormove
colormove_EXTERNAL_OBJECTS =

devel/lib/image_pkg/colormove: CMakeFiles/colormove.dir/src/colormove.cpp.o
devel/lib/image_pkg/colormove: CMakeFiles/colormove.dir/build.make
devel/lib/image_pkg/colormove: /opt/ros/melodic/lib/libimage_transport.so
devel/lib/image_pkg/colormove: /opt/ros/melodic/lib/libmessage_filters.so
devel/lib/image_pkg/colormove: /opt/ros/melodic/lib/libclass_loader.so
devel/lib/image_pkg/colormove: /usr/lib/libPocoFoundation.so
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libdl.so
devel/lib/image_pkg/colormove: /opt/ros/melodic/lib/libroscpp.so
devel/lib/image_pkg/colormove: /opt/ros/melodic/lib/libxmlrpcpp.so
devel/lib/image_pkg/colormove: /opt/ros/melodic/lib/libroslib.so
devel/lib/image_pkg/colormove: /opt/ros/melodic/lib/librospack.so
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libpython2.7.so
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
devel/lib/image_pkg/colormove: /opt/ros/melodic/lib/libcv_bridge.so
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
devel/lib/image_pkg/colormove: /opt/ros/melodic/lib/librosconsole.so
devel/lib/image_pkg/colormove: /opt/ros/melodic/lib/librosconsole_log4cxx.so
devel/lib/image_pkg/colormove: /opt/ros/melodic/lib/librosconsole_backend_interface.so
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/image_pkg/colormove: /opt/ros/melodic/lib/libroscpp_serialization.so
devel/lib/image_pkg/colormove: /opt/ros/melodic/lib/librostime.so
devel/lib/image_pkg/colormove: /opt/ros/melodic/lib/libcpp_common.so
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
devel/lib/image_pkg/colormove: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
devel/lib/image_pkg/colormove: CMakeFiles/colormove.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zhuzhengming/dip_ws/src/image_pkg/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable devel/lib/image_pkg/colormove"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/colormove.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/colormove.dir/build: devel/lib/image_pkg/colormove
.PHONY : CMakeFiles/colormove.dir/build

CMakeFiles/colormove.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/colormove.dir/cmake_clean.cmake
.PHONY : CMakeFiles/colormove.dir/clean

CMakeFiles/colormove.dir/depend:
	cd /home/zhuzhengming/dip_ws/src/image_pkg/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zhuzhengming/dip_ws/src/image_pkg /home/zhuzhengming/dip_ws/src/image_pkg /home/zhuzhengming/dip_ws/src/image_pkg/cmake-build-debug /home/zhuzhengming/dip_ws/src/image_pkg/cmake-build-debug /home/zhuzhengming/dip_ws/src/image_pkg/cmake-build-debug/CMakeFiles/colormove.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/colormove.dir/depend
