# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/build

# Utility rule file for check-typekit-uptodate.

# Include the progress variables for this target.
include typekit/CMakeFiles/check-typekit-uptodate.dir/progress.make

typekit/CMakeFiles/check-typekit-uptodate: ../typekit/stamp

../typekit/stamp: ../typekit/../include/ProactiveAssistance/ProactiveAssistance-types.hpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Typekit input changed. Run make regen in your build directory first"
	cd /home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/build/typekit && /bin/false

check-typekit-uptodate: typekit/CMakeFiles/check-typekit-uptodate
check-typekit-uptodate: ../typekit/stamp
check-typekit-uptodate: typekit/CMakeFiles/check-typekit-uptodate.dir/build.make
.PHONY : check-typekit-uptodate

# Rule to build all files generated by this target.
typekit/CMakeFiles/check-typekit-uptodate.dir/build: check-typekit-uptodate
.PHONY : typekit/CMakeFiles/check-typekit-uptodate.dir/build

typekit/CMakeFiles/check-typekit-uptodate.dir/clean:
	cd /home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/build/typekit && $(CMAKE_COMMAND) -P CMakeFiles/check-typekit-uptodate.dir/cmake_clean.cmake
.PHONY : typekit/CMakeFiles/check-typekit-uptodate.dir/clean

typekit/CMakeFiles/check-typekit-uptodate.dir/depend:
	cd /home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance /home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/typekit /home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/build /home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/build/typekit /home/intelligentrobotics/ws/orocos/Applications/ProactiveAssistance/build/typekit/CMakeFiles/check-typekit-uptodate.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : typekit/CMakeFiles/check-typekit-uptodate.dir/depend

