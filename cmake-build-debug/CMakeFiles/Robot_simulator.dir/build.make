# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = /snap/clion/126/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/126/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jake/Desktop/SLAM-Methods

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jake/Desktop/SLAM-Methods/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/Robot_simulator.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Robot_simulator.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Robot_simulator.dir/flags.make

CMakeFiles/Robot_simulator.dir/main.cpp.o: CMakeFiles/Robot_simulator.dir/flags.make
CMakeFiles/Robot_simulator.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jake/Desktop/SLAM-Methods/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Robot_simulator.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Robot_simulator.dir/main.cpp.o -c /home/jake/Desktop/SLAM-Methods/main.cpp

CMakeFiles/Robot_simulator.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Robot_simulator.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jake/Desktop/SLAM-Methods/main.cpp > CMakeFiles/Robot_simulator.dir/main.cpp.i

CMakeFiles/Robot_simulator.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Robot_simulator.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jake/Desktop/SLAM-Methods/main.cpp -o CMakeFiles/Robot_simulator.dir/main.cpp.s

# Object files for target Robot_simulator
Robot_simulator_OBJECTS = \
"CMakeFiles/Robot_simulator.dir/main.cpp.o"

# External object files for target Robot_simulator
Robot_simulator_EXTERNAL_OBJECTS =

Robot_simulator: CMakeFiles/Robot_simulator.dir/main.cpp.o
Robot_simulator: CMakeFiles/Robot_simulator.dir/build.make
Robot_simulator: /usr/lib/python2.7/config-x86_64-linux-gnu/libpython2.7.so
Robot_simulator: CMakeFiles/Robot_simulator.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jake/Desktop/SLAM-Methods/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Robot_simulator"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Robot_simulator.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Robot_simulator.dir/build: Robot_simulator

.PHONY : CMakeFiles/Robot_simulator.dir/build

CMakeFiles/Robot_simulator.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Robot_simulator.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Robot_simulator.dir/clean

CMakeFiles/Robot_simulator.dir/depend:
	cd /home/jake/Desktop/SLAM-Methods/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jake/Desktop/SLAM-Methods /home/jake/Desktop/SLAM-Methods /home/jake/Desktop/SLAM-Methods/cmake-build-debug /home/jake/Desktop/SLAM-Methods/cmake-build-debug /home/jake/Desktop/SLAM-Methods/cmake-build-debug/CMakeFiles/Robot_simulator.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Robot_simulator.dir/depend

