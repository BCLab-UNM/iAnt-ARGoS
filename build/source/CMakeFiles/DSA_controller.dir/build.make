# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.3

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.3.1/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.3.1/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/linhtran/Desktop/iAnt-ARGoS-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/linhtran/Desktop/iAnt-ARGoS-master/build

# Include any dependencies generated for this target.
include source/CMakeFiles/DSA_controller.dir/depend.make

# Include the progress variables for this target.
include source/CMakeFiles/DSA_controller.dir/progress.make

# Include the compile flags for this target's objects.
include source/CMakeFiles/DSA_controller.dir/flags.make

source/CMakeFiles/DSA_controller.dir/DSA_controller.cpp.o: source/CMakeFiles/DSA_controller.dir/flags.make
source/CMakeFiles/DSA_controller.dir/DSA_controller.cpp.o: ../source/DSA_controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/linhtran/Desktop/iAnt-ARGoS-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object source/CMakeFiles/DSA_controller.dir/DSA_controller.cpp.o"
	cd /Users/linhtran/Desktop/iAnt-ARGoS-master/build/source && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/DSA_controller.dir/DSA_controller.cpp.o -c /Users/linhtran/Desktop/iAnt-ARGoS-master/source/DSA_controller.cpp

source/CMakeFiles/DSA_controller.dir/DSA_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DSA_controller.dir/DSA_controller.cpp.i"
	cd /Users/linhtran/Desktop/iAnt-ARGoS-master/build/source && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/linhtran/Desktop/iAnt-ARGoS-master/source/DSA_controller.cpp > CMakeFiles/DSA_controller.dir/DSA_controller.cpp.i

source/CMakeFiles/DSA_controller.dir/DSA_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DSA_controller.dir/DSA_controller.cpp.s"
	cd /Users/linhtran/Desktop/iAnt-ARGoS-master/build/source && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/linhtran/Desktop/iAnt-ARGoS-master/source/DSA_controller.cpp -o CMakeFiles/DSA_controller.dir/DSA_controller.cpp.s

source/CMakeFiles/DSA_controller.dir/DSA_controller.cpp.o.requires:

.PHONY : source/CMakeFiles/DSA_controller.dir/DSA_controller.cpp.o.requires

source/CMakeFiles/DSA_controller.dir/DSA_controller.cpp.o.provides: source/CMakeFiles/DSA_controller.dir/DSA_controller.cpp.o.requires
	$(MAKE) -f source/CMakeFiles/DSA_controller.dir/build.make source/CMakeFiles/DSA_controller.dir/DSA_controller.cpp.o.provides.build
.PHONY : source/CMakeFiles/DSA_controller.dir/DSA_controller.cpp.o.provides

source/CMakeFiles/DSA_controller.dir/DSA_controller.cpp.o.provides.build: source/CMakeFiles/DSA_controller.dir/DSA_controller.cpp.o


source/CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.o: source/CMakeFiles/DSA_controller.dir/flags.make
source/CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.o: ../source/iAnt_loop_functions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/linhtran/Desktop/iAnt-ARGoS-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object source/CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.o"
	cd /Users/linhtran/Desktop/iAnt-ARGoS-master/build/source && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.o -c /Users/linhtran/Desktop/iAnt-ARGoS-master/source/iAnt_loop_functions.cpp

source/CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.i"
	cd /Users/linhtran/Desktop/iAnt-ARGoS-master/build/source && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/linhtran/Desktop/iAnt-ARGoS-master/source/iAnt_loop_functions.cpp > CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.i

source/CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.s"
	cd /Users/linhtran/Desktop/iAnt-ARGoS-master/build/source && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/linhtran/Desktop/iAnt-ARGoS-master/source/iAnt_loop_functions.cpp -o CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.s

source/CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.o.requires:

.PHONY : source/CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.o.requires

source/CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.o.provides: source/CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.o.requires
	$(MAKE) -f source/CMakeFiles/DSA_controller.dir/build.make source/CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.o.provides.build
.PHONY : source/CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.o.provides

source/CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.o.provides.build: source/CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.o


source/CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.o: source/CMakeFiles/DSA_controller.dir/flags.make
source/CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.o: ../source/iAnt_pheromone.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/linhtran/Desktop/iAnt-ARGoS-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object source/CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.o"
	cd /Users/linhtran/Desktop/iAnt-ARGoS-master/build/source && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.o -c /Users/linhtran/Desktop/iAnt-ARGoS-master/source/iAnt_pheromone.cpp

source/CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.i"
	cd /Users/linhtran/Desktop/iAnt-ARGoS-master/build/source && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/linhtran/Desktop/iAnt-ARGoS-master/source/iAnt_pheromone.cpp > CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.i

source/CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.s"
	cd /Users/linhtran/Desktop/iAnt-ARGoS-master/build/source && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/linhtran/Desktop/iAnt-ARGoS-master/source/iAnt_pheromone.cpp -o CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.s

source/CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.o.requires:

.PHONY : source/CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.o.requires

source/CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.o.provides: source/CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.o.requires
	$(MAKE) -f source/CMakeFiles/DSA_controller.dir/build.make source/CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.o.provides.build
.PHONY : source/CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.o.provides

source/CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.o.provides.build: source/CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.o


# Object files for target DSA_controller
DSA_controller_OBJECTS = \
"CMakeFiles/DSA_controller.dir/DSA_controller.cpp.o" \
"CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.o" \
"CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.o"

# External object files for target DSA_controller
DSA_controller_EXTERNAL_OBJECTS =

source/libDSA_controller.so: source/CMakeFiles/DSA_controller.dir/DSA_controller.cpp.o
source/libDSA_controller.so: source/CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.o
source/libDSA_controller.so: source/CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.o
source/libDSA_controller.so: source/CMakeFiles/DSA_controller.dir/build.make
source/libDSA_controller.so: source/CMakeFiles/DSA_controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/linhtran/Desktop/iAnt-ARGoS-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared module libDSA_controller.so"
	cd /Users/linhtran/Desktop/iAnt-ARGoS-master/build/source && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/DSA_controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
source/CMakeFiles/DSA_controller.dir/build: source/libDSA_controller.so

.PHONY : source/CMakeFiles/DSA_controller.dir/build

source/CMakeFiles/DSA_controller.dir/requires: source/CMakeFiles/DSA_controller.dir/DSA_controller.cpp.o.requires
source/CMakeFiles/DSA_controller.dir/requires: source/CMakeFiles/DSA_controller.dir/iAnt_loop_functions.cpp.o.requires
source/CMakeFiles/DSA_controller.dir/requires: source/CMakeFiles/DSA_controller.dir/iAnt_pheromone.cpp.o.requires

.PHONY : source/CMakeFiles/DSA_controller.dir/requires

source/CMakeFiles/DSA_controller.dir/clean:
	cd /Users/linhtran/Desktop/iAnt-ARGoS-master/build/source && $(CMAKE_COMMAND) -P CMakeFiles/DSA_controller.dir/cmake_clean.cmake
.PHONY : source/CMakeFiles/DSA_controller.dir/clean

source/CMakeFiles/DSA_controller.dir/depend:
	cd /Users/linhtran/Desktop/iAnt-ARGoS-master/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/linhtran/Desktop/iAnt-ARGoS-master /Users/linhtran/Desktop/iAnt-ARGoS-master/source /Users/linhtran/Desktop/iAnt-ARGoS-master/build /Users/linhtran/Desktop/iAnt-ARGoS-master/build/source /Users/linhtran/Desktop/iAnt-ARGoS-master/build/source/CMakeFiles/DSA_controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : source/CMakeFiles/DSA_controller.dir/depend

