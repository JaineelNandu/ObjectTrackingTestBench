# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = "/home/jaineelnandu/Documents/Object Tracking/Scheduler System - Incomplete (04-07-2021)/CompleteSystem/cpptrial"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/jaineelnandu/Documents/Object Tracking/Scheduler System - Incomplete (04-07-2021)/CompleteSystem/cpptrial"

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start "/home/jaineelnandu/Documents/Object Tracking/Scheduler System - Incomplete (04-07-2021)/CompleteSystem/cpptrial/CMakeFiles" "/home/jaineelnandu/Documents/Object Tracking/Scheduler System - Incomplete (04-07-2021)/CompleteSystem/cpptrial/CMakeFiles/progress.marks"
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start "/home/jaineelnandu/Documents/Object Tracking/Scheduler System - Incomplete (04-07-2021)/CompleteSystem/cpptrial/CMakeFiles" 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named executeObsGenTests

# Build rule for target.
executeObsGenTests: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 executeObsGenTests
.PHONY : executeObsGenTests

# fast build rule for target.
executeObsGenTests/fast:
	$(MAKE) -f CMakeFiles/executeObsGenTests.dir/build.make CMakeFiles/executeObsGenTests.dir/build
.PHONY : executeObsGenTests/fast

#=============================================================================
# Target rules for targets named executeTests

# Build rule for target.
executeTests: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 executeTests
.PHONY : executeTests

# fast build rule for target.
executeTests/fast:
	$(MAKE) -f CMakeFiles/executeTests.dir/build.make CMakeFiles/executeTests.dir/build
.PHONY : executeTests/fast

#=============================================================================
# Target rules for targets named executeVerletTests

# Build rule for target.
executeVerletTests: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 executeVerletTests
.PHONY : executeVerletTests

# fast build rule for target.
executeVerletTests/fast:
	$(MAKE) -f CMakeFiles/executeVerletTests.dir/build.make CMakeFiles/executeVerletTests.dir/build
.PHONY : executeVerletTests/fast

ObsGenerator_test.o: ObsGenerator_test.cpp.o

.PHONY : ObsGenerator_test.o

# target to build an object file
ObsGenerator_test.cpp.o:
	$(MAKE) -f CMakeFiles/executeObsGenTests.dir/build.make CMakeFiles/executeObsGenTests.dir/ObsGenerator_test.cpp.o
.PHONY : ObsGenerator_test.cpp.o

ObsGenerator_test.i: ObsGenerator_test.cpp.i

.PHONY : ObsGenerator_test.i

# target to preprocess a source file
ObsGenerator_test.cpp.i:
	$(MAKE) -f CMakeFiles/executeObsGenTests.dir/build.make CMakeFiles/executeObsGenTests.dir/ObsGenerator_test.cpp.i
.PHONY : ObsGenerator_test.cpp.i

ObsGenerator_test.s: ObsGenerator_test.cpp.s

.PHONY : ObsGenerator_test.s

# target to generate assembly for a file
ObsGenerator_test.cpp.s:
	$(MAKE) -f CMakeFiles/executeObsGenTests.dir/build.make CMakeFiles/executeObsGenTests.dir/ObsGenerator_test.cpp.s
.PHONY : ObsGenerator_test.cpp.s

VerletIntegration_test.o: VerletIntegration_test.cpp.o

.PHONY : VerletIntegration_test.o

# target to build an object file
VerletIntegration_test.cpp.o:
	$(MAKE) -f CMakeFiles/executeVerletTests.dir/build.make CMakeFiles/executeVerletTests.dir/VerletIntegration_test.cpp.o
.PHONY : VerletIntegration_test.cpp.o

VerletIntegration_test.i: VerletIntegration_test.cpp.i

.PHONY : VerletIntegration_test.i

# target to preprocess a source file
VerletIntegration_test.cpp.i:
	$(MAKE) -f CMakeFiles/executeVerletTests.dir/build.make CMakeFiles/executeVerletTests.dir/VerletIntegration_test.cpp.i
.PHONY : VerletIntegration_test.cpp.i

VerletIntegration_test.s: VerletIntegration_test.cpp.s

.PHONY : VerletIntegration_test.s

# target to generate assembly for a file
VerletIntegration_test.cpp.s:
	$(MAKE) -f CMakeFiles/executeVerletTests.dir/build.make CMakeFiles/executeVerletTests.dir/VerletIntegration_test.cpp.s
.PHONY : VerletIntegration_test.cpp.s

sqrt_test.o: sqrt_test.cpp.o

.PHONY : sqrt_test.o

# target to build an object file
sqrt_test.cpp.o:
	$(MAKE) -f CMakeFiles/executeTests.dir/build.make CMakeFiles/executeTests.dir/sqrt_test.cpp.o
.PHONY : sqrt_test.cpp.o

sqrt_test.i: sqrt_test.cpp.i

.PHONY : sqrt_test.i

# target to preprocess a source file
sqrt_test.cpp.i:
	$(MAKE) -f CMakeFiles/executeTests.dir/build.make CMakeFiles/executeTests.dir/sqrt_test.cpp.i
.PHONY : sqrt_test.cpp.i

sqrt_test.s: sqrt_test.cpp.s

.PHONY : sqrt_test.s

# target to generate assembly for a file
sqrt_test.cpp.s:
	$(MAKE) -f CMakeFiles/executeTests.dir/build.make CMakeFiles/executeTests.dir/sqrt_test.cpp.s
.PHONY : sqrt_test.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... executeObsGenTests"
	@echo "... executeTests"
	@echo "... executeVerletTests"
	@echo "... rebuild_cache"
	@echo "... ObsGenerator_test.o"
	@echo "... ObsGenerator_test.i"
	@echo "... ObsGenerator_test.s"
	@echo "... VerletIntegration_test.o"
	@echo "... VerletIntegration_test.i"
	@echo "... VerletIntegration_test.s"
	@echo "... sqrt_test.o"
	@echo "... sqrt_test.i"
	@echo "... sqrt_test.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

