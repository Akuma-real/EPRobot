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
include simple_navigation_goals/CMakeFiles/plan_bk_a.dir/depend.make

# Include the progress variables for this target.
include simple_navigation_goals/CMakeFiles/plan_bk_a.dir/progress.make

# Include the compile flags for this target's objects.
include simple_navigation_goals/CMakeFiles/plan_bk_a.dir/flags.make

simple_navigation_goals/CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.o: simple_navigation_goals/CMakeFiles/plan_bk_a.dir/flags.make
simple_navigation_goals/CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.o: /home/EPRobot/robot_ws/src/simple_navigation_goals/src/plan_bk_a.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/EPRobot/robot_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object simple_navigation_goals/CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.o"
	cd /home/EPRobot/robot_ws/build/simple_navigation_goals && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.o -c /home/EPRobot/robot_ws/src/simple_navigation_goals/src/plan_bk_a.cpp

simple_navigation_goals/CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.i"
	cd /home/EPRobot/robot_ws/build/simple_navigation_goals && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/EPRobot/robot_ws/src/simple_navigation_goals/src/plan_bk_a.cpp > CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.i

simple_navigation_goals/CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.s"
	cd /home/EPRobot/robot_ws/build/simple_navigation_goals && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/EPRobot/robot_ws/src/simple_navigation_goals/src/plan_bk_a.cpp -o CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.s

simple_navigation_goals/CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.o.requires:

.PHONY : simple_navigation_goals/CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.o.requires

simple_navigation_goals/CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.o.provides: simple_navigation_goals/CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.o.requires
	$(MAKE) -f simple_navigation_goals/CMakeFiles/plan_bk_a.dir/build.make simple_navigation_goals/CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.o.provides.build
.PHONY : simple_navigation_goals/CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.o.provides

simple_navigation_goals/CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.o.provides.build: simple_navigation_goals/CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.o


# Object files for target plan_bk_a
plan_bk_a_OBJECTS = \
"CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.o"

# External object files for target plan_bk_a
plan_bk_a_EXTERNAL_OBJECTS =

/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: simple_navigation_goals/CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.o
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: simple_navigation_goals/CMakeFiles/plan_bk_a.dir/build.make
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /opt/ros/melodic/lib/libtf.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /opt/ros/melodic/lib/libtf2_ros.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /opt/ros/melodic/lib/libactionlib.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /opt/ros/melodic/lib/libmessage_filters.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /opt/ros/melodic/lib/libroscpp.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /opt/ros/melodic/lib/libtf2.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /opt/ros/melodic/lib/librosconsole.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /opt/ros/melodic/lib/librostime.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /opt/ros/melodic/lib/libcpp_common.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a: simple_navigation_goals/CMakeFiles/plan_bk_a.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/EPRobot/robot_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a"
	cd /home/EPRobot/robot_ws/build/simple_navigation_goals && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/plan_bk_a.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
simple_navigation_goals/CMakeFiles/plan_bk_a.dir/build: /home/EPRobot/robot_ws/devel/lib/simple_navigation_goals/plan_bk_a

.PHONY : simple_navigation_goals/CMakeFiles/plan_bk_a.dir/build

simple_navigation_goals/CMakeFiles/plan_bk_a.dir/requires: simple_navigation_goals/CMakeFiles/plan_bk_a.dir/src/plan_bk_a.cpp.o.requires

.PHONY : simple_navigation_goals/CMakeFiles/plan_bk_a.dir/requires

simple_navigation_goals/CMakeFiles/plan_bk_a.dir/clean:
	cd /home/EPRobot/robot_ws/build/simple_navigation_goals && $(CMAKE_COMMAND) -P CMakeFiles/plan_bk_a.dir/cmake_clean.cmake
.PHONY : simple_navigation_goals/CMakeFiles/plan_bk_a.dir/clean

simple_navigation_goals/CMakeFiles/plan_bk_a.dir/depend:
	cd /home/EPRobot/robot_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/EPRobot/robot_ws/src /home/EPRobot/robot_ws/src/simple_navigation_goals /home/EPRobot/robot_ws/build /home/EPRobot/robot_ws/build/simple_navigation_goals /home/EPRobot/robot_ws/build/simple_navigation_goals/CMakeFiles/plan_bk_a.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : simple_navigation_goals/CMakeFiles/plan_bk_a.dir/depend

