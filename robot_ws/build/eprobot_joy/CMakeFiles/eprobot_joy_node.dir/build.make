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
include eprobot_joy/CMakeFiles/eprobot_joy_node.dir/depend.make

# Include the progress variables for this target.
include eprobot_joy/CMakeFiles/eprobot_joy_node.dir/progress.make

# Include the compile flags for this target's objects.
include eprobot_joy/CMakeFiles/eprobot_joy_node.dir/flags.make

eprobot_joy/CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.o: eprobot_joy/CMakeFiles/eprobot_joy_node.dir/flags.make
eprobot_joy/CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.o: /home/EPRobot/robot_ws/src/eprobot_joy/src/eprobot_joy.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/EPRobot/robot_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object eprobot_joy/CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.o"
	cd /home/EPRobot/robot_ws/build/eprobot_joy && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.o -c /home/EPRobot/robot_ws/src/eprobot_joy/src/eprobot_joy.cpp

eprobot_joy/CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.i"
	cd /home/EPRobot/robot_ws/build/eprobot_joy && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/EPRobot/robot_ws/src/eprobot_joy/src/eprobot_joy.cpp > CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.i

eprobot_joy/CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.s"
	cd /home/EPRobot/robot_ws/build/eprobot_joy && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/EPRobot/robot_ws/src/eprobot_joy/src/eprobot_joy.cpp -o CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.s

eprobot_joy/CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.o.requires:

.PHONY : eprobot_joy/CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.o.requires

eprobot_joy/CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.o.provides: eprobot_joy/CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.o.requires
	$(MAKE) -f eprobot_joy/CMakeFiles/eprobot_joy_node.dir/build.make eprobot_joy/CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.o.provides.build
.PHONY : eprobot_joy/CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.o.provides

eprobot_joy/CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.o.provides.build: eprobot_joy/CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.o


# Object files for target eprobot_joy_node
eprobot_joy_node_OBJECTS = \
"CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.o"

# External object files for target eprobot_joy_node
eprobot_joy_node_EXTERNAL_OBJECTS =

/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: eprobot_joy/CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.o
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: eprobot_joy/CMakeFiles/eprobot_joy_node.dir/build.make
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /opt/ros/melodic/lib/libserial.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /opt/ros/melodic/lib/libroslib.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /opt/ros/melodic/lib/librospack.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /usr/lib/aarch64-linux-gnu/libpython2.7.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /opt/ros/melodic/lib/libtf.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /opt/ros/melodic/lib/libtf2_ros.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /opt/ros/melodic/lib/libactionlib.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /opt/ros/melodic/lib/libmessage_filters.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /opt/ros/melodic/lib/libroscpp.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /opt/ros/melodic/lib/libtf2.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /opt/ros/melodic/lib/librosconsole.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /opt/ros/melodic/lib/librostime.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /opt/ros/melodic/lib/libcpp_common.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node: eprobot_joy/CMakeFiles/eprobot_joy_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/EPRobot/robot_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node"
	cd /home/EPRobot/robot_ws/build/eprobot_joy && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/eprobot_joy_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
eprobot_joy/CMakeFiles/eprobot_joy_node.dir/build: /home/EPRobot/robot_ws/devel/lib/eprobot_joy/eprobot_joy_node

.PHONY : eprobot_joy/CMakeFiles/eprobot_joy_node.dir/build

eprobot_joy/CMakeFiles/eprobot_joy_node.dir/requires: eprobot_joy/CMakeFiles/eprobot_joy_node.dir/src/eprobot_joy.cpp.o.requires

.PHONY : eprobot_joy/CMakeFiles/eprobot_joy_node.dir/requires

eprobot_joy/CMakeFiles/eprobot_joy_node.dir/clean:
	cd /home/EPRobot/robot_ws/build/eprobot_joy && $(CMAKE_COMMAND) -P CMakeFiles/eprobot_joy_node.dir/cmake_clean.cmake
.PHONY : eprobot_joy/CMakeFiles/eprobot_joy_node.dir/clean

eprobot_joy/CMakeFiles/eprobot_joy_node.dir/depend:
	cd /home/EPRobot/robot_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/EPRobot/robot_ws/src /home/EPRobot/robot_ws/src/eprobot_joy /home/EPRobot/robot_ws/build /home/EPRobot/robot_ws/build/eprobot_joy /home/EPRobot/robot_ws/build/eprobot_joy/CMakeFiles/eprobot_joy_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : eprobot_joy/CMakeFiles/eprobot_joy_node.dir/depend
