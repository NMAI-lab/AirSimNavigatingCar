# CMAKE generated file: DO NOT EDIT!
# Generated by "NMake Makefiles" Generator, CMake Version 3.15

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

!IF "$(OS)" == "Windows_NT"
NULL=
!ELSE
NULL=nul
!ENDIF
SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = C:\opt\python27amd64\Lib\site-packages\cmake\data\bin\cmake.exe

# The command to remove a file.
RM = C:\opt\python27amd64\Lib\site-packages\cmake\data\bin\cmake.exe -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\james\catkin_ws\src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\james\catkin_ws\build

# Utility rule file for _catkin_empty_exported_target.

# Include the progress variables for this target.
include airsim\CMakeFiles\_catkin_empty_exported_target.dir\progress.make

_catkin_empty_exported_target: airsim\CMakeFiles\_catkin_empty_exported_target.dir\build.make

.PHONY : _catkin_empty_exported_target

# Rule to build all files generated by this target.
airsim\CMakeFiles\_catkin_empty_exported_target.dir\build: _catkin_empty_exported_target

.PHONY : airsim\CMakeFiles\_catkin_empty_exported_target.dir\build

airsim\CMakeFiles\_catkin_empty_exported_target.dir\clean:
	cd C:\Users\james\catkin_ws\build\airsim
	$(CMAKE_COMMAND) -P CMakeFiles\_catkin_empty_exported_target.dir\cmake_clean.cmake
	cd C:\Users\james\catkin_ws\build
.PHONY : airsim\CMakeFiles\_catkin_empty_exported_target.dir\clean

airsim\CMakeFiles\_catkin_empty_exported_target.dir\depend:
	$(CMAKE_COMMAND) -E cmake_depends "NMake Makefiles" C:\Users\james\catkin_ws\src C:\Users\james\catkin_ws\src\airsim C:\Users\james\catkin_ws\build C:\Users\james\catkin_ws\build\airsim C:\Users\james\catkin_ws\build\airsim\CMakeFiles\_catkin_empty_exported_target.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : airsim\CMakeFiles\_catkin_empty_exported_target.dir\depend

