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
CMAKE_COMMAND = /opt/cmake-3.16/bin/cmake

# The command to remove a file.
RM = /opt/cmake-3.16/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/shekoufeh/Desktop/FOV-Optimization-on-Manifold-main/Manifold_cpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shekoufeh/Desktop/FOV-Optimization-on-Manifold-main/Manifold_cpp/mybuild

# Include any dependencies generated for this target.
include manifold/CMakeFiles/manifold.dir/depend.make

# Include the progress variables for this target.
include manifold/CMakeFiles/manifold.dir/progress.make

# Include the compile flags for this target's objects.
include manifold/CMakeFiles/manifold.dir/flags.make

# Object files for target manifold
manifold_OBJECTS =

# External object files for target manifold
manifold_EXTERNAL_OBJECTS =

manifold/libmanifold.a: manifold/CMakeFiles/manifold.dir/build.make
manifold/libmanifold.a: manifold/CMakeFiles/manifold.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/shekoufeh/Desktop/FOV-Optimization-on-Manifold-main/Manifold_cpp/mybuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Linking CXX static library libmanifold.a"
	cd /home/shekoufeh/Desktop/FOV-Optimization-on-Manifold-main/Manifold_cpp/mybuild/manifold && $(CMAKE_COMMAND) -P CMakeFiles/manifold.dir/cmake_clean_target.cmake
	cd /home/shekoufeh/Desktop/FOV-Optimization-on-Manifold-main/Manifold_cpp/mybuild/manifold && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/manifold.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
manifold/CMakeFiles/manifold.dir/build: manifold/libmanifold.a

.PHONY : manifold/CMakeFiles/manifold.dir/build

manifold/CMakeFiles/manifold.dir/clean:
	cd /home/shekoufeh/Desktop/FOV-Optimization-on-Manifold-main/Manifold_cpp/mybuild/manifold && $(CMAKE_COMMAND) -P CMakeFiles/manifold.dir/cmake_clean.cmake
.PHONY : manifold/CMakeFiles/manifold.dir/clean

manifold/CMakeFiles/manifold.dir/depend:
	cd /home/shekoufeh/Desktop/FOV-Optimization-on-Manifold-main/Manifold_cpp/mybuild && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shekoufeh/Desktop/FOV-Optimization-on-Manifold-main/Manifold_cpp /home/shekoufeh/Desktop/FOV-Optimization-on-Manifold-main/Manifold_cpp/manifold /home/shekoufeh/Desktop/FOV-Optimization-on-Manifold-main/Manifold_cpp/mybuild /home/shekoufeh/Desktop/FOV-Optimization-on-Manifold-main/Manifold_cpp/mybuild/manifold /home/shekoufeh/Desktop/FOV-Optimization-on-Manifold-main/Manifold_cpp/mybuild/manifold/CMakeFiles/manifold.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : manifold/CMakeFiles/manifold.dir/depend

