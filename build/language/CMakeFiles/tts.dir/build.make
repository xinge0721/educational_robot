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
CMAKE_SOURCE_DIR = /home/ros/chenxu/educational_robot/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ros/chenxu/educational_robot/build

# Include any dependencies generated for this target.
include language/CMakeFiles/tts.dir/depend.make

# Include the progress variables for this target.
include language/CMakeFiles/tts.dir/progress.make

# Include the compile flags for this target's objects.
include language/CMakeFiles/tts.dir/flags.make

language/CMakeFiles/tts.dir/src/tts/tts_init.cpp.o: language/CMakeFiles/tts.dir/flags.make
language/CMakeFiles/tts.dir/src/tts/tts_init.cpp.o: /home/ros/chenxu/educational_robot/src/language/src/tts/tts_init.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/chenxu/educational_robot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object language/CMakeFiles/tts.dir/src/tts/tts_init.cpp.o"
	cd /home/ros/chenxu/educational_robot/build/language && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tts.dir/src/tts/tts_init.cpp.o -c /home/ros/chenxu/educational_robot/src/language/src/tts/tts_init.cpp

language/CMakeFiles/tts.dir/src/tts/tts_init.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tts.dir/src/tts/tts_init.cpp.i"
	cd /home/ros/chenxu/educational_robot/build/language && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/chenxu/educational_robot/src/language/src/tts/tts_init.cpp > CMakeFiles/tts.dir/src/tts/tts_init.cpp.i

language/CMakeFiles/tts.dir/src/tts/tts_init.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tts.dir/src/tts/tts_init.cpp.s"
	cd /home/ros/chenxu/educational_robot/build/language && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/chenxu/educational_robot/src/language/src/tts/tts_init.cpp -o CMakeFiles/tts.dir/src/tts/tts_init.cpp.s

# Object files for target tts
tts_OBJECTS = \
"CMakeFiles/tts.dir/src/tts/tts_init.cpp.o"

# External object files for target tts
tts_EXTERNAL_OBJECTS =

/home/ros/chenxu/educational_robot/devel/lib/language/tts: language/CMakeFiles/tts.dir/src/tts/tts_init.cpp.o
/home/ros/chenxu/educational_robot/devel/lib/language/tts: language/CMakeFiles/tts.dir/build.make
/home/ros/chenxu/educational_robot/devel/lib/language/tts: /opt/ros/noetic/lib/libroscpp.so
/home/ros/chenxu/educational_robot/devel/lib/language/tts: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/ros/chenxu/educational_robot/devel/lib/language/tts: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/ros/chenxu/educational_robot/devel/lib/language/tts: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/ros/chenxu/educational_robot/devel/lib/language/tts: /opt/ros/noetic/lib/librosconsole.so
/home/ros/chenxu/educational_robot/devel/lib/language/tts: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/ros/chenxu/educational_robot/devel/lib/language/tts: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/ros/chenxu/educational_robot/devel/lib/language/tts: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/ros/chenxu/educational_robot/devel/lib/language/tts: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/ros/chenxu/educational_robot/devel/lib/language/tts: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/ros/chenxu/educational_robot/devel/lib/language/tts: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/ros/chenxu/educational_robot/devel/lib/language/tts: /opt/ros/noetic/lib/librostime.so
/home/ros/chenxu/educational_robot/devel/lib/language/tts: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/ros/chenxu/educational_robot/devel/lib/language/tts: /opt/ros/noetic/lib/libcpp_common.so
/home/ros/chenxu/educational_robot/devel/lib/language/tts: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/ros/chenxu/educational_robot/devel/lib/language/tts: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/ros/chenxu/educational_robot/devel/lib/language/tts: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/ros/chenxu/educational_robot/devel/lib/language/tts: language/CMakeFiles/tts.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ros/chenxu/educational_robot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/ros/chenxu/educational_robot/devel/lib/language/tts"
	cd /home/ros/chenxu/educational_robot/build/language && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tts.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
language/CMakeFiles/tts.dir/build: /home/ros/chenxu/educational_robot/devel/lib/language/tts

.PHONY : language/CMakeFiles/tts.dir/build

language/CMakeFiles/tts.dir/clean:
	cd /home/ros/chenxu/educational_robot/build/language && $(CMAKE_COMMAND) -P CMakeFiles/tts.dir/cmake_clean.cmake
.PHONY : language/CMakeFiles/tts.dir/clean

language/CMakeFiles/tts.dir/depend:
	cd /home/ros/chenxu/educational_robot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros/chenxu/educational_robot/src /home/ros/chenxu/educational_robot/src/language /home/ros/chenxu/educational_robot/build /home/ros/chenxu/educational_robot/build/language /home/ros/chenxu/educational_robot/build/language/CMakeFiles/tts.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : language/CMakeFiles/tts.dir/depend

