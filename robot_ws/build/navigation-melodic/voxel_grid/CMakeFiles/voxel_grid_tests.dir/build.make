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
CMAKE_SOURCE_DIR = /home/EPRobot/robot_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/EPRobot/robot_ws/build

# Include any dependencies generated for this target.
include navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/depend.make

# Include the progress variables for this target.
include navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/progress.make

# Include the compile flags for this target's objects.
include navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/flags.make

navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o: navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/flags.make
navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o: /home/EPRobot/robot_ws/src/navigation-melodic/voxel_grid/test/voxel_grid_tests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/EPRobot/robot_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o"
	cd /home/EPRobot/robot_ws/build/navigation-melodic/voxel_grid && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o -c /home/EPRobot/robot_ws/src/navigation-melodic/voxel_grid/test/voxel_grid_tests.cpp

navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.i"
	cd /home/EPRobot/robot_ws/build/navigation-melodic/voxel_grid && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/EPRobot/robot_ws/src/navigation-melodic/voxel_grid/test/voxel_grid_tests.cpp > CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.i

navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.s"
	cd /home/EPRobot/robot_ws/build/navigation-melodic/voxel_grid && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/EPRobot/robot_ws/src/navigation-melodic/voxel_grid/test/voxel_grid_tests.cpp -o CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.s

navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o.requires:

.PHONY : navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o.requires

navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o.provides: navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o.requires
	$(MAKE) -f navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/build.make navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o.provides.build
.PHONY : navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o.provides

navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o.provides.build: navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o


# Object files for target voxel_grid_tests
voxel_grid_tests_OBJECTS = \
"CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o"

# External object files for target voxel_grid_tests
voxel_grid_tests_EXTERNAL_OBJECTS =

/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/build.make
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: gtest/googlemock/gtest/libgtest.so
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: /home/EPRobot/robot_ws/devel/lib/libvoxel_grid.so
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: /opt/ros/melodic/lib/libroscpp.so
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: /opt/ros/melodic/lib/librosconsole.so
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: /opt/ros/melodic/lib/librostime.so
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: /opt/ros/melodic/lib/libcpp_common.so
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests: navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/EPRobot/robot_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests"
	cd /home/EPRobot/robot_ws/build/navigation-melodic/voxel_grid && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/voxel_grid_tests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/build: /home/EPRobot/robot_ws/devel/lib/voxel_grid/voxel_grid_tests

.PHONY : navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/build

navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/requires: navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/test/voxel_grid_tests.cpp.o.requires

.PHONY : navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/requires

navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/clean:
	cd /home/EPRobot/robot_ws/build/navigation-melodic/voxel_grid && $(CMAKE_COMMAND) -P CMakeFiles/voxel_grid_tests.dir/cmake_clean.cmake
.PHONY : navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/clean

navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/depend:
	cd /home/EPRobot/robot_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/EPRobot/robot_ws/src /home/EPRobot/robot_ws/src/navigation-melodic/voxel_grid /home/EPRobot/robot_ws/build /home/EPRobot/robot_ws/build/navigation-melodic/voxel_grid /home/EPRobot/robot_ws/build/navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : navigation-melodic/voxel_grid/CMakeFiles/voxel_grid_tests.dir/depend

