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
CMAKE_SOURCE_DIR = /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build

# Include any dependencies generated for this target.
include modules/highgui/CMakeFiles/opencv_highgui.dir/depend.make

# Include the progress variables for this target.
include modules/highgui/CMakeFiles/opencv_highgui.dir/progress.make

# Include the compile flags for this target's objects.
include modules/highgui/CMakeFiles/opencv_highgui.dir/flags.make

modules/highgui/CMakeFiles/opencv_highgui.dir/src/window.cpp.o: modules/highgui/CMakeFiles/opencv_highgui.dir/flags.make
modules/highgui/CMakeFiles/opencv_highgui.dir/src/window.cpp.o: ../modules/highgui/src/window.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object modules/highgui/CMakeFiles/opencv_highgui.dir/src/window.cpp.o"
	cd /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build/modules/highgui && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/opencv_highgui.dir/src/window.cpp.o -c /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/modules/highgui/src/window.cpp

modules/highgui/CMakeFiles/opencv_highgui.dir/src/window.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_highgui.dir/src/window.cpp.i"
	cd /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build/modules/highgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/modules/highgui/src/window.cpp > CMakeFiles/opencv_highgui.dir/src/window.cpp.i

modules/highgui/CMakeFiles/opencv_highgui.dir/src/window.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_highgui.dir/src/window.cpp.s"
	cd /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build/modules/highgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/modules/highgui/src/window.cpp -o CMakeFiles/opencv_highgui.dir/src/window.cpp.s

modules/highgui/CMakeFiles/opencv_highgui.dir/src/window.cpp.o.requires:

.PHONY : modules/highgui/CMakeFiles/opencv_highgui.dir/src/window.cpp.o.requires

modules/highgui/CMakeFiles/opencv_highgui.dir/src/window.cpp.o.provides: modules/highgui/CMakeFiles/opencv_highgui.dir/src/window.cpp.o.requires
	$(MAKE) -f modules/highgui/CMakeFiles/opencv_highgui.dir/build.make modules/highgui/CMakeFiles/opencv_highgui.dir/src/window.cpp.o.provides.build
.PHONY : modules/highgui/CMakeFiles/opencv_highgui.dir/src/window.cpp.o.provides

modules/highgui/CMakeFiles/opencv_highgui.dir/src/window.cpp.o.provides.build: modules/highgui/CMakeFiles/opencv_highgui.dir/src/window.cpp.o


modules/highgui/CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.o: modules/highgui/CMakeFiles/opencv_highgui.dir/flags.make
modules/highgui/CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.o: ../modules/highgui/src/roiSelector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object modules/highgui/CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.o"
	cd /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build/modules/highgui && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.o -c /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/modules/highgui/src/roiSelector.cpp

modules/highgui/CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.i"
	cd /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build/modules/highgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/modules/highgui/src/roiSelector.cpp > CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.i

modules/highgui/CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.s"
	cd /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build/modules/highgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/modules/highgui/src/roiSelector.cpp -o CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.s

modules/highgui/CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.o.requires:

.PHONY : modules/highgui/CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.o.requires

modules/highgui/CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.o.provides: modules/highgui/CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.o.requires
	$(MAKE) -f modules/highgui/CMakeFiles/opencv_highgui.dir/build.make modules/highgui/CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.o.provides.build
.PHONY : modules/highgui/CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.o.provides

modules/highgui/CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.o.provides.build: modules/highgui/CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.o


modules/highgui/CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.o: modules/highgui/CMakeFiles/opencv_highgui.dir/flags.make
modules/highgui/CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.o: ../modules/highgui/src/window_gtk.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object modules/highgui/CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.o"
	cd /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build/modules/highgui && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.o -c /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/modules/highgui/src/window_gtk.cpp

modules/highgui/CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.i"
	cd /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build/modules/highgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/modules/highgui/src/window_gtk.cpp > CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.i

modules/highgui/CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.s"
	cd /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build/modules/highgui && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/modules/highgui/src/window_gtk.cpp -o CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.s

modules/highgui/CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.o.requires:

.PHONY : modules/highgui/CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.o.requires

modules/highgui/CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.o.provides: modules/highgui/CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.o.requires
	$(MAKE) -f modules/highgui/CMakeFiles/opencv_highgui.dir/build.make modules/highgui/CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.o.provides.build
.PHONY : modules/highgui/CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.o.provides

modules/highgui/CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.o.provides.build: modules/highgui/CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.o


# Object files for target opencv_highgui
opencv_highgui_OBJECTS = \
"CMakeFiles/opencv_highgui.dir/src/window.cpp.o" \
"CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.o" \
"CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.o"

# External object files for target opencv_highgui
opencv_highgui_EXTERNAL_OBJECTS =

lib/libopencv_highgui.so.3.4.16: modules/highgui/CMakeFiles/opencv_highgui.dir/src/window.cpp.o
lib/libopencv_highgui.so.3.4.16: modules/highgui/CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.o
lib/libopencv_highgui.so.3.4.16: modules/highgui/CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.o
lib/libopencv_highgui.so.3.4.16: modules/highgui/CMakeFiles/opencv_highgui.dir/build.make
lib/libopencv_highgui.so.3.4.16: lib/libopencv_videoio.so.3.4.16
lib/libopencv_highgui.so.3.4.16: /usr/lib/x86_64-linux-gnu/libgtk-x11-2.0.so
lib/libopencv_highgui.so.3.4.16: /usr/lib/x86_64-linux-gnu/libgdk-x11-2.0.so
lib/libopencv_highgui.so.3.4.16: /usr/lib/x86_64-linux-gnu/libpangocairo-1.0.so
lib/libopencv_highgui.so.3.4.16: /usr/lib/x86_64-linux-gnu/libatk-1.0.so
lib/libopencv_highgui.so.3.4.16: /usr/lib/x86_64-linux-gnu/libcairo.so
lib/libopencv_highgui.so.3.4.16: /usr/lib/x86_64-linux-gnu/libgdk_pixbuf-2.0.so
lib/libopencv_highgui.so.3.4.16: /usr/lib/x86_64-linux-gnu/libgio-2.0.so
lib/libopencv_highgui.so.3.4.16: /usr/lib/x86_64-linux-gnu/libpangoft2-1.0.so
lib/libopencv_highgui.so.3.4.16: /usr/lib/x86_64-linux-gnu/libpango-1.0.so
lib/libopencv_highgui.so.3.4.16: /usr/lib/x86_64-linux-gnu/libgobject-2.0.so
lib/libopencv_highgui.so.3.4.16: /usr/lib/x86_64-linux-gnu/libglib-2.0.so
lib/libopencv_highgui.so.3.4.16: /usr/lib/x86_64-linux-gnu/libfontconfig.so
lib/libopencv_highgui.so.3.4.16: /usr/lib/x86_64-linux-gnu/libfreetype.so
lib/libopencv_highgui.so.3.4.16: /usr/lib/x86_64-linux-gnu/libgthread-2.0.so
lib/libopencv_highgui.so.3.4.16: lib/libopencv_imgcodecs.so.3.4.16
lib/libopencv_highgui.so.3.4.16: lib/libopencv_imgproc.so.3.4.16
lib/libopencv_highgui.so.3.4.16: lib/libopencv_core.so.3.4.16
lib/libopencv_highgui.so.3.4.16: modules/highgui/CMakeFiles/opencv_highgui.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library ../../lib/libopencv_highgui.so"
	cd /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build/modules/highgui && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/opencv_highgui.dir/link.txt --verbose=$(VERBOSE)
	cd /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build/modules/highgui && $(CMAKE_COMMAND) -E cmake_symlink_library ../../lib/libopencv_highgui.so.3.4.16 ../../lib/libopencv_highgui.so.3.4 ../../lib/libopencv_highgui.so

lib/libopencv_highgui.so.3.4: lib/libopencv_highgui.so.3.4.16
	@$(CMAKE_COMMAND) -E touch_nocreate lib/libopencv_highgui.so.3.4

lib/libopencv_highgui.so: lib/libopencv_highgui.so.3.4.16
	@$(CMAKE_COMMAND) -E touch_nocreate lib/libopencv_highgui.so

# Rule to build all files generated by this target.
modules/highgui/CMakeFiles/opencv_highgui.dir/build: lib/libopencv_highgui.so

.PHONY : modules/highgui/CMakeFiles/opencv_highgui.dir/build

modules/highgui/CMakeFiles/opencv_highgui.dir/requires: modules/highgui/CMakeFiles/opencv_highgui.dir/src/window.cpp.o.requires
modules/highgui/CMakeFiles/opencv_highgui.dir/requires: modules/highgui/CMakeFiles/opencv_highgui.dir/src/roiSelector.cpp.o.requires
modules/highgui/CMakeFiles/opencv_highgui.dir/requires: modules/highgui/CMakeFiles/opencv_highgui.dir/src/window_gtk.cpp.o.requires

.PHONY : modules/highgui/CMakeFiles/opencv_highgui.dir/requires

modules/highgui/CMakeFiles/opencv_highgui.dir/clean:
	cd /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build/modules/highgui && $(CMAKE_COMMAND) -P CMakeFiles/opencv_highgui.dir/cmake_clean.cmake
.PHONY : modules/highgui/CMakeFiles/opencv_highgui.dir/clean

modules/highgui/CMakeFiles/opencv_highgui.dir/depend:
	cd /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16 /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/modules/highgui /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build/modules/highgui /home/wzs/Graphics/RealTimeRendering/instructs/opencv-3.4.16/build/modules/highgui/CMakeFiles/opencv_highgui.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : modules/highgui/CMakeFiles/opencv_highgui.dir/depend

