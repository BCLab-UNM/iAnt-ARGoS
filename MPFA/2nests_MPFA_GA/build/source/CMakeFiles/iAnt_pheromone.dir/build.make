# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.4

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.4.0/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.4.0/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/qilu/Documents/UNM/S8/Research/GA/2016_02_14_GA_code/2nests_MPFA_GA

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/qilu/Documents/UNM/S8/Research/GA/2016_02_14_GA_code/2nests_MPFA_GA/build

# Include any dependencies generated for this target.
include source/CMakeFiles/iAnt_pheromone.dir/depend.make

# Include the progress variables for this target.
include source/CMakeFiles/iAnt_pheromone.dir/progress.make

# Include the compile flags for this target's objects.
include source/CMakeFiles/iAnt_pheromone.dir/flags.make

source/CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.o: source/CMakeFiles/iAnt_pheromone.dir/flags.make
source/CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.o: ../source/iAnt_pheromone.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/qilu/Documents/UNM/S8/Research/GA/2016_02_14_GA_code/2nests_MPFA_GA/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object source/CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.o"
	cd /Users/qilu/Documents/UNM/S8/Research/GA/2016_02_14_GA_code/2nests_MPFA_GA/build/source && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.o -c /Users/qilu/Documents/UNM/S8/Research/GA/2016_02_14_GA_code/2nests_MPFA_GA/source/iAnt_pheromone.cpp

source/CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.i"
	cd /Users/qilu/Documents/UNM/S8/Research/GA/2016_02_14_GA_code/2nests_MPFA_GA/build/source && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/qilu/Documents/UNM/S8/Research/GA/2016_02_14_GA_code/2nests_MPFA_GA/source/iAnt_pheromone.cpp > CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.i

source/CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.s"
	cd /Users/qilu/Documents/UNM/S8/Research/GA/2016_02_14_GA_code/2nests_MPFA_GA/build/source && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/qilu/Documents/UNM/S8/Research/GA/2016_02_14_GA_code/2nests_MPFA_GA/source/iAnt_pheromone.cpp -o CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.s

source/CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.o.requires:

.PHONY : source/CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.o.requires

source/CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.o.provides: source/CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.o.requires
	$(MAKE) -f source/CMakeFiles/iAnt_pheromone.dir/build.make source/CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.o.provides.build
.PHONY : source/CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.o.provides

source/CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.o.provides.build: source/CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.o


# Object files for target iAnt_pheromone
iAnt_pheromone_OBJECTS = \
"CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.o"

# External object files for target iAnt_pheromone
iAnt_pheromone_EXTERNAL_OBJECTS =

source/libiAnt_pheromone.dylib: source/CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.o
source/libiAnt_pheromone.dylib: source/CMakeFiles/iAnt_pheromone.dir/build.make
source/libiAnt_pheromone.dylib: source/CMakeFiles/iAnt_pheromone.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/qilu/Documents/UNM/S8/Research/GA/2016_02_14_GA_code/2nests_MPFA_GA/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libiAnt_pheromone.dylib"
	cd /Users/qilu/Documents/UNM/S8/Research/GA/2016_02_14_GA_code/2nests_MPFA_GA/build/source && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/iAnt_pheromone.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
source/CMakeFiles/iAnt_pheromone.dir/build: source/libiAnt_pheromone.dylib

.PHONY : source/CMakeFiles/iAnt_pheromone.dir/build

source/CMakeFiles/iAnt_pheromone.dir/requires: source/CMakeFiles/iAnt_pheromone.dir/iAnt_pheromone.cpp.o.requires

.PHONY : source/CMakeFiles/iAnt_pheromone.dir/requires

source/CMakeFiles/iAnt_pheromone.dir/clean:
	cd /Users/qilu/Documents/UNM/S8/Research/GA/2016_02_14_GA_code/2nests_MPFA_GA/build/source && $(CMAKE_COMMAND) -P CMakeFiles/iAnt_pheromone.dir/cmake_clean.cmake
.PHONY : source/CMakeFiles/iAnt_pheromone.dir/clean

source/CMakeFiles/iAnt_pheromone.dir/depend:
	cd /Users/qilu/Documents/UNM/S8/Research/GA/2016_02_14_GA_code/2nests_MPFA_GA/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/qilu/Documents/UNM/S8/Research/GA/2016_02_14_GA_code/2nests_MPFA_GA /Users/qilu/Documents/UNM/S8/Research/GA/2016_02_14_GA_code/2nests_MPFA_GA/source /Users/qilu/Documents/UNM/S8/Research/GA/2016_02_14_GA_code/2nests_MPFA_GA/build /Users/qilu/Documents/UNM/S8/Research/GA/2016_02_14_GA_code/2nests_MPFA_GA/build/source /Users/qilu/Documents/UNM/S8/Research/GA/2016_02_14_GA_code/2nests_MPFA_GA/build/source/CMakeFiles/iAnt_pheromone.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : source/CMakeFiles/iAnt_pheromone.dir/depend

