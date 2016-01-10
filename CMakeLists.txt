cmake_minimum_required(VERSION 2.8.12)
project(iAnt-ARGoS)

# Use C+11
set(CMAKE_CXX_FLAGS "-std=c++0x")

# Deactivate RPATH for MacOSX.
set(CMAKE_MACOSX_RPATH 0)

# Set the path where CMake will find additional scripts.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Find the ARGoS package.
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)

# Check for required libraries to compile code depending on Qt and OpenGL.
include(ARGoSCheckQTOpenGL)

# Find the GSL package.
find_package(GSL REQUIRED)

# Find the LUA package.
find_package(Lua52 REQUIRED)

# Set ARGoS include directory.
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${GSL_INCLUDE_DIR} ${LUA_INCLUDE_DIR})

# Set ARGoS link directory.
link_directories(${ARGOS_LIBRARY_DIRS})

# Descend into the source code directory.
add_subdirectory(source)
