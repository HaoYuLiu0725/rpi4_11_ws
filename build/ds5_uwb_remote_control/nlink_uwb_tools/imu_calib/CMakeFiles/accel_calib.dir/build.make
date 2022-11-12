# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/haoyu/rpi4_11_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/haoyu/rpi4_11_ws/build

# Include any dependencies generated for this target.
include ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/CMakeFiles/accel_calib.dir/depend.make

# Include the progress variables for this target.
include ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/CMakeFiles/accel_calib.dir/progress.make

# Include the compile flags for this target's objects.
include ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/CMakeFiles/accel_calib.dir/flags.make

ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/CMakeFiles/accel_calib.dir/src/accel_calib/accel_calib.cpp.o: ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/CMakeFiles/accel_calib.dir/flags.make
ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/CMakeFiles/accel_calib.dir/src/accel_calib/accel_calib.cpp.o: /home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/src/accel_calib/accel_calib.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/haoyu/rpi4_11_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/CMakeFiles/accel_calib.dir/src/accel_calib/accel_calib.cpp.o"
	cd /home/haoyu/rpi4_11_ws/build/ds5_uwb_remote_control/nlink_uwb_tools/imu_calib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/accel_calib.dir/src/accel_calib/accel_calib.cpp.o -c /home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/src/accel_calib/accel_calib.cpp

ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/CMakeFiles/accel_calib.dir/src/accel_calib/accel_calib.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/accel_calib.dir/src/accel_calib/accel_calib.cpp.i"
	cd /home/haoyu/rpi4_11_ws/build/ds5_uwb_remote_control/nlink_uwb_tools/imu_calib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/src/accel_calib/accel_calib.cpp > CMakeFiles/accel_calib.dir/src/accel_calib/accel_calib.cpp.i

ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/CMakeFiles/accel_calib.dir/src/accel_calib/accel_calib.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/accel_calib.dir/src/accel_calib/accel_calib.cpp.s"
	cd /home/haoyu/rpi4_11_ws/build/ds5_uwb_remote_control/nlink_uwb_tools/imu_calib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/src/accel_calib/accel_calib.cpp -o CMakeFiles/accel_calib.dir/src/accel_calib/accel_calib.cpp.s

# Object files for target accel_calib
accel_calib_OBJECTS = \
"CMakeFiles/accel_calib.dir/src/accel_calib/accel_calib.cpp.o"

# External object files for target accel_calib
accel_calib_EXTERNAL_OBJECTS =

/home/haoyu/rpi4_11_ws/devel/lib/libaccel_calib.so: ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/CMakeFiles/accel_calib.dir/src/accel_calib/accel_calib.cpp.o
/home/haoyu/rpi4_11_ws/devel/lib/libaccel_calib.so: ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/CMakeFiles/accel_calib.dir/build.make
/home/haoyu/rpi4_11_ws/devel/lib/libaccel_calib.so: ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/CMakeFiles/accel_calib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/haoyu/rpi4_11_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/haoyu/rpi4_11_ws/devel/lib/libaccel_calib.so"
	cd /home/haoyu/rpi4_11_ws/build/ds5_uwb_remote_control/nlink_uwb_tools/imu_calib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/accel_calib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/CMakeFiles/accel_calib.dir/build: /home/haoyu/rpi4_11_ws/devel/lib/libaccel_calib.so

.PHONY : ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/CMakeFiles/accel_calib.dir/build

ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/CMakeFiles/accel_calib.dir/clean:
	cd /home/haoyu/rpi4_11_ws/build/ds5_uwb_remote_control/nlink_uwb_tools/imu_calib && $(CMAKE_COMMAND) -P CMakeFiles/accel_calib.dir/cmake_clean.cmake
.PHONY : ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/CMakeFiles/accel_calib.dir/clean

ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/CMakeFiles/accel_calib.dir/depend:
	cd /home/haoyu/rpi4_11_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/haoyu/rpi4_11_ws/src /home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/imu_calib /home/haoyu/rpi4_11_ws/build /home/haoyu/rpi4_11_ws/build/ds5_uwb_remote_control/nlink_uwb_tools/imu_calib /home/haoyu/rpi4_11_ws/build/ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/CMakeFiles/accel_calib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ds5_uwb_remote_control/nlink_uwb_tools/imu_calib/CMakeFiles/accel_calib.dir/depend

