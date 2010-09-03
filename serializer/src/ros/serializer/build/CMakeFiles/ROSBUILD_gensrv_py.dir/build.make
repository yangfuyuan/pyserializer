# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.6

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/patrick/Eclipse/serializer/src/ros/serializer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/patrick/Eclipse/serializer/src/ros/serializer/build

# Utility rule file for ROSBUILD_gensrv_py.

CMakeFiles/ROSBUILD_gensrv_py: ../src/serializer/srv/__init__.py

../src/serializer/srv/__init__.py: ../src/serializer/srv/_TravelDistance.py
../src/serializer/srv/__init__.py: ../src/serializer/srv/_GetAnalog.py
../src/serializer/srv/__init__.py: ../src/serializer/srv/_GetDigital.py
../src/serializer/srv/__init__.py: ../src/serializer/srv/_SetDigital.py
../src/serializer/srv/__init__.py: ../src/serializer/srv/_IsMoving.py
../src/serializer/srv/__init__.py: ../src/serializer/srv/_RotateAngle.py
	$(CMAKE_COMMAND) -E cmake_progress_report /home/patrick/Eclipse/serializer/src/ros/serializer/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/serializer/srv/__init__.py"
	/home/patrick/ros/ros/core/rospy/scripts/gensrv_py --initpy /home/patrick/Eclipse/serializer/src/ros/serializer/srv/TravelDistance.srv /home/patrick/Eclipse/serializer/src/ros/serializer/srv/GetAnalog.srv /home/patrick/Eclipse/serializer/src/ros/serializer/srv/GetDigital.srv /home/patrick/Eclipse/serializer/src/ros/serializer/srv/SetDigital.srv /home/patrick/Eclipse/serializer/src/ros/serializer/srv/IsMoving.srv /home/patrick/Eclipse/serializer/src/ros/serializer/srv/RotateAngle.srv

../src/serializer/srv/_TravelDistance.py: ../srv/TravelDistance.srv
../src/serializer/srv/_TravelDistance.py: /home/patrick/ros/ros/core/rospy/scripts/gensrv_py
../src/serializer/srv/_TravelDistance.py: /home/patrick/ros/ros/core/roslib/scripts/gendeps
../src/serializer/srv/_TravelDistance.py: ../manifest.xml
../src/serializer/srv/_TravelDistance.py: /home/patrick/ros/ros/core/genmsg_cpp/manifest.xml
../src/serializer/srv/_TravelDistance.py: /home/patrick/ros/ros/tools/rospack/manifest.xml
../src/serializer/srv/_TravelDistance.py: /home/patrick/ros/ros/core/roslib/manifest.xml
../src/serializer/srv/_TravelDistance.py: /home/patrick/ros/ros/std_msgs/manifest.xml
../src/serializer/srv/_TravelDistance.py: /home/patrick/ros/ros/core/roslang/manifest.xml
../src/serializer/srv/_TravelDistance.py: /home/patrick/ros/ros/core/rospy/manifest.xml
../src/serializer/srv/_TravelDistance.py: /home/patrick/ros/ros/3rdparty/xmlrpcpp/manifest.xml
../src/serializer/srv/_TravelDistance.py: /home/patrick/ros/ros/core/rosconsole/manifest.xml
../src/serializer/srv/_TravelDistance.py: /home/patrick/ros/ros/core/roscpp/manifest.xml
../src/serializer/srv/_TravelDistance.py: /home/patrick/ros/ros/3rdparty/pycrypto/manifest.xml
../src/serializer/srv/_TravelDistance.py: /home/patrick/ros/ros/3rdparty/paramiko/manifest.xml
../src/serializer/srv/_TravelDistance.py: /home/patrick/ros/ros/core/rosout/manifest.xml
../src/serializer/srv/_TravelDistance.py: /home/patrick/ros/ros/tools/roslaunch/manifest.xml
../src/serializer/srv/_TravelDistance.py: /home/patrick/ros/ros/test/rostest/manifest.xml
../src/serializer/srv/_TravelDistance.py: /home/patrick/ros/ros/tools/topic_tools/manifest.xml
../src/serializer/srv/_TravelDistance.py: /home/patrick/ros/ros/tools/rosrecord/manifest.xml
../src/serializer/srv/_TravelDistance.py: /home/patrick/ros/ros/tools/rosbagmigration/manifest.xml
../src/serializer/srv/_TravelDistance.py: /opt/ros/boxturtle/stacks/common_msgs/geometry_msgs/manifest.xml
../src/serializer/srv/_TravelDistance.py: /opt/ros/boxturtle/stacks/common_msgs/sensor_msgs/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/patrick/Eclipse/serializer/src/ros/serializer/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/serializer/srv/_TravelDistance.py"
	/home/patrick/ros/ros/core/rospy/scripts/gensrv_py --noinitpy /home/patrick/Eclipse/serializer/src/ros/serializer/srv/TravelDistance.srv

../src/serializer/srv/_GetAnalog.py: ../srv/GetAnalog.srv
../src/serializer/srv/_GetAnalog.py: /home/patrick/ros/ros/core/rospy/scripts/gensrv_py
../src/serializer/srv/_GetAnalog.py: /home/patrick/ros/ros/core/roslib/scripts/gendeps
../src/serializer/srv/_GetAnalog.py: ../manifest.xml
../src/serializer/srv/_GetAnalog.py: /home/patrick/ros/ros/core/genmsg_cpp/manifest.xml
../src/serializer/srv/_GetAnalog.py: /home/patrick/ros/ros/tools/rospack/manifest.xml
../src/serializer/srv/_GetAnalog.py: /home/patrick/ros/ros/core/roslib/manifest.xml
../src/serializer/srv/_GetAnalog.py: /home/patrick/ros/ros/std_msgs/manifest.xml
../src/serializer/srv/_GetAnalog.py: /home/patrick/ros/ros/core/roslang/manifest.xml
../src/serializer/srv/_GetAnalog.py: /home/patrick/ros/ros/core/rospy/manifest.xml
../src/serializer/srv/_GetAnalog.py: /home/patrick/ros/ros/3rdparty/xmlrpcpp/manifest.xml
../src/serializer/srv/_GetAnalog.py: /home/patrick/ros/ros/core/rosconsole/manifest.xml
../src/serializer/srv/_GetAnalog.py: /home/patrick/ros/ros/core/roscpp/manifest.xml
../src/serializer/srv/_GetAnalog.py: /home/patrick/ros/ros/3rdparty/pycrypto/manifest.xml
../src/serializer/srv/_GetAnalog.py: /home/patrick/ros/ros/3rdparty/paramiko/manifest.xml
../src/serializer/srv/_GetAnalog.py: /home/patrick/ros/ros/core/rosout/manifest.xml
../src/serializer/srv/_GetAnalog.py: /home/patrick/ros/ros/tools/roslaunch/manifest.xml
../src/serializer/srv/_GetAnalog.py: /home/patrick/ros/ros/test/rostest/manifest.xml
../src/serializer/srv/_GetAnalog.py: /home/patrick/ros/ros/tools/topic_tools/manifest.xml
../src/serializer/srv/_GetAnalog.py: /home/patrick/ros/ros/tools/rosrecord/manifest.xml
../src/serializer/srv/_GetAnalog.py: /home/patrick/ros/ros/tools/rosbagmigration/manifest.xml
../src/serializer/srv/_GetAnalog.py: /opt/ros/boxturtle/stacks/common_msgs/geometry_msgs/manifest.xml
../src/serializer/srv/_GetAnalog.py: /opt/ros/boxturtle/stacks/common_msgs/sensor_msgs/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/patrick/Eclipse/serializer/src/ros/serializer/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/serializer/srv/_GetAnalog.py"
	/home/patrick/ros/ros/core/rospy/scripts/gensrv_py --noinitpy /home/patrick/Eclipse/serializer/src/ros/serializer/srv/GetAnalog.srv

../src/serializer/srv/_GetDigital.py: ../srv/GetDigital.srv
../src/serializer/srv/_GetDigital.py: /home/patrick/ros/ros/core/rospy/scripts/gensrv_py
../src/serializer/srv/_GetDigital.py: /home/patrick/ros/ros/core/roslib/scripts/gendeps
../src/serializer/srv/_GetDigital.py: ../manifest.xml
../src/serializer/srv/_GetDigital.py: /home/patrick/ros/ros/core/genmsg_cpp/manifest.xml
../src/serializer/srv/_GetDigital.py: /home/patrick/ros/ros/tools/rospack/manifest.xml
../src/serializer/srv/_GetDigital.py: /home/patrick/ros/ros/core/roslib/manifest.xml
../src/serializer/srv/_GetDigital.py: /home/patrick/ros/ros/std_msgs/manifest.xml
../src/serializer/srv/_GetDigital.py: /home/patrick/ros/ros/core/roslang/manifest.xml
../src/serializer/srv/_GetDigital.py: /home/patrick/ros/ros/core/rospy/manifest.xml
../src/serializer/srv/_GetDigital.py: /home/patrick/ros/ros/3rdparty/xmlrpcpp/manifest.xml
../src/serializer/srv/_GetDigital.py: /home/patrick/ros/ros/core/rosconsole/manifest.xml
../src/serializer/srv/_GetDigital.py: /home/patrick/ros/ros/core/roscpp/manifest.xml
../src/serializer/srv/_GetDigital.py: /home/patrick/ros/ros/3rdparty/pycrypto/manifest.xml
../src/serializer/srv/_GetDigital.py: /home/patrick/ros/ros/3rdparty/paramiko/manifest.xml
../src/serializer/srv/_GetDigital.py: /home/patrick/ros/ros/core/rosout/manifest.xml
../src/serializer/srv/_GetDigital.py: /home/patrick/ros/ros/tools/roslaunch/manifest.xml
../src/serializer/srv/_GetDigital.py: /home/patrick/ros/ros/test/rostest/manifest.xml
../src/serializer/srv/_GetDigital.py: /home/patrick/ros/ros/tools/topic_tools/manifest.xml
../src/serializer/srv/_GetDigital.py: /home/patrick/ros/ros/tools/rosrecord/manifest.xml
../src/serializer/srv/_GetDigital.py: /home/patrick/ros/ros/tools/rosbagmigration/manifest.xml
../src/serializer/srv/_GetDigital.py: /opt/ros/boxturtle/stacks/common_msgs/geometry_msgs/manifest.xml
../src/serializer/srv/_GetDigital.py: /opt/ros/boxturtle/stacks/common_msgs/sensor_msgs/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/patrick/Eclipse/serializer/src/ros/serializer/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/serializer/srv/_GetDigital.py"
	/home/patrick/ros/ros/core/rospy/scripts/gensrv_py --noinitpy /home/patrick/Eclipse/serializer/src/ros/serializer/srv/GetDigital.srv

../src/serializer/srv/_SetDigital.py: ../srv/SetDigital.srv
../src/serializer/srv/_SetDigital.py: /home/patrick/ros/ros/core/rospy/scripts/gensrv_py
../src/serializer/srv/_SetDigital.py: /home/patrick/ros/ros/core/roslib/scripts/gendeps
../src/serializer/srv/_SetDigital.py: ../manifest.xml
../src/serializer/srv/_SetDigital.py: /home/patrick/ros/ros/core/genmsg_cpp/manifest.xml
../src/serializer/srv/_SetDigital.py: /home/patrick/ros/ros/tools/rospack/manifest.xml
../src/serializer/srv/_SetDigital.py: /home/patrick/ros/ros/core/roslib/manifest.xml
../src/serializer/srv/_SetDigital.py: /home/patrick/ros/ros/std_msgs/manifest.xml
../src/serializer/srv/_SetDigital.py: /home/patrick/ros/ros/core/roslang/manifest.xml
../src/serializer/srv/_SetDigital.py: /home/patrick/ros/ros/core/rospy/manifest.xml
../src/serializer/srv/_SetDigital.py: /home/patrick/ros/ros/3rdparty/xmlrpcpp/manifest.xml
../src/serializer/srv/_SetDigital.py: /home/patrick/ros/ros/core/rosconsole/manifest.xml
../src/serializer/srv/_SetDigital.py: /home/patrick/ros/ros/core/roscpp/manifest.xml
../src/serializer/srv/_SetDigital.py: /home/patrick/ros/ros/3rdparty/pycrypto/manifest.xml
../src/serializer/srv/_SetDigital.py: /home/patrick/ros/ros/3rdparty/paramiko/manifest.xml
../src/serializer/srv/_SetDigital.py: /home/patrick/ros/ros/core/rosout/manifest.xml
../src/serializer/srv/_SetDigital.py: /home/patrick/ros/ros/tools/roslaunch/manifest.xml
../src/serializer/srv/_SetDigital.py: /home/patrick/ros/ros/test/rostest/manifest.xml
../src/serializer/srv/_SetDigital.py: /home/patrick/ros/ros/tools/topic_tools/manifest.xml
../src/serializer/srv/_SetDigital.py: /home/patrick/ros/ros/tools/rosrecord/manifest.xml
../src/serializer/srv/_SetDigital.py: /home/patrick/ros/ros/tools/rosbagmigration/manifest.xml
../src/serializer/srv/_SetDigital.py: /opt/ros/boxturtle/stacks/common_msgs/geometry_msgs/manifest.xml
../src/serializer/srv/_SetDigital.py: /opt/ros/boxturtle/stacks/common_msgs/sensor_msgs/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/patrick/Eclipse/serializer/src/ros/serializer/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/serializer/srv/_SetDigital.py"
	/home/patrick/ros/ros/core/rospy/scripts/gensrv_py --noinitpy /home/patrick/Eclipse/serializer/src/ros/serializer/srv/SetDigital.srv

../src/serializer/srv/_IsMoving.py: ../srv/IsMoving.srv
../src/serializer/srv/_IsMoving.py: /home/patrick/ros/ros/core/rospy/scripts/gensrv_py
../src/serializer/srv/_IsMoving.py: /home/patrick/ros/ros/core/roslib/scripts/gendeps
../src/serializer/srv/_IsMoving.py: ../manifest.xml
../src/serializer/srv/_IsMoving.py: /home/patrick/ros/ros/core/genmsg_cpp/manifest.xml
../src/serializer/srv/_IsMoving.py: /home/patrick/ros/ros/tools/rospack/manifest.xml
../src/serializer/srv/_IsMoving.py: /home/patrick/ros/ros/core/roslib/manifest.xml
../src/serializer/srv/_IsMoving.py: /home/patrick/ros/ros/std_msgs/manifest.xml
../src/serializer/srv/_IsMoving.py: /home/patrick/ros/ros/core/roslang/manifest.xml
../src/serializer/srv/_IsMoving.py: /home/patrick/ros/ros/core/rospy/manifest.xml
../src/serializer/srv/_IsMoving.py: /home/patrick/ros/ros/3rdparty/xmlrpcpp/manifest.xml
../src/serializer/srv/_IsMoving.py: /home/patrick/ros/ros/core/rosconsole/manifest.xml
../src/serializer/srv/_IsMoving.py: /home/patrick/ros/ros/core/roscpp/manifest.xml
../src/serializer/srv/_IsMoving.py: /home/patrick/ros/ros/3rdparty/pycrypto/manifest.xml
../src/serializer/srv/_IsMoving.py: /home/patrick/ros/ros/3rdparty/paramiko/manifest.xml
../src/serializer/srv/_IsMoving.py: /home/patrick/ros/ros/core/rosout/manifest.xml
../src/serializer/srv/_IsMoving.py: /home/patrick/ros/ros/tools/roslaunch/manifest.xml
../src/serializer/srv/_IsMoving.py: /home/patrick/ros/ros/test/rostest/manifest.xml
../src/serializer/srv/_IsMoving.py: /home/patrick/ros/ros/tools/topic_tools/manifest.xml
../src/serializer/srv/_IsMoving.py: /home/patrick/ros/ros/tools/rosrecord/manifest.xml
../src/serializer/srv/_IsMoving.py: /home/patrick/ros/ros/tools/rosbagmigration/manifest.xml
../src/serializer/srv/_IsMoving.py: /opt/ros/boxturtle/stacks/common_msgs/geometry_msgs/manifest.xml
../src/serializer/srv/_IsMoving.py: /opt/ros/boxturtle/stacks/common_msgs/sensor_msgs/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/patrick/Eclipse/serializer/src/ros/serializer/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/serializer/srv/_IsMoving.py"
	/home/patrick/ros/ros/core/rospy/scripts/gensrv_py --noinitpy /home/patrick/Eclipse/serializer/src/ros/serializer/srv/IsMoving.srv

../src/serializer/srv/_RotateAngle.py: ../srv/RotateAngle.srv
../src/serializer/srv/_RotateAngle.py: /home/patrick/ros/ros/core/rospy/scripts/gensrv_py
../src/serializer/srv/_RotateAngle.py: /home/patrick/ros/ros/core/roslib/scripts/gendeps
../src/serializer/srv/_RotateAngle.py: ../manifest.xml
../src/serializer/srv/_RotateAngle.py: /home/patrick/ros/ros/core/genmsg_cpp/manifest.xml
../src/serializer/srv/_RotateAngle.py: /home/patrick/ros/ros/tools/rospack/manifest.xml
../src/serializer/srv/_RotateAngle.py: /home/patrick/ros/ros/core/roslib/manifest.xml
../src/serializer/srv/_RotateAngle.py: /home/patrick/ros/ros/std_msgs/manifest.xml
../src/serializer/srv/_RotateAngle.py: /home/patrick/ros/ros/core/roslang/manifest.xml
../src/serializer/srv/_RotateAngle.py: /home/patrick/ros/ros/core/rospy/manifest.xml
../src/serializer/srv/_RotateAngle.py: /home/patrick/ros/ros/3rdparty/xmlrpcpp/manifest.xml
../src/serializer/srv/_RotateAngle.py: /home/patrick/ros/ros/core/rosconsole/manifest.xml
../src/serializer/srv/_RotateAngle.py: /home/patrick/ros/ros/core/roscpp/manifest.xml
../src/serializer/srv/_RotateAngle.py: /home/patrick/ros/ros/3rdparty/pycrypto/manifest.xml
../src/serializer/srv/_RotateAngle.py: /home/patrick/ros/ros/3rdparty/paramiko/manifest.xml
../src/serializer/srv/_RotateAngle.py: /home/patrick/ros/ros/core/rosout/manifest.xml
../src/serializer/srv/_RotateAngle.py: /home/patrick/ros/ros/tools/roslaunch/manifest.xml
../src/serializer/srv/_RotateAngle.py: /home/patrick/ros/ros/test/rostest/manifest.xml
../src/serializer/srv/_RotateAngle.py: /home/patrick/ros/ros/tools/topic_tools/manifest.xml
../src/serializer/srv/_RotateAngle.py: /home/patrick/ros/ros/tools/rosrecord/manifest.xml
../src/serializer/srv/_RotateAngle.py: /home/patrick/ros/ros/tools/rosbagmigration/manifest.xml
../src/serializer/srv/_RotateAngle.py: /opt/ros/boxturtle/stacks/common_msgs/geometry_msgs/manifest.xml
../src/serializer/srv/_RotateAngle.py: /opt/ros/boxturtle/stacks/common_msgs/sensor_msgs/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/patrick/Eclipse/serializer/src/ros/serializer/build/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/serializer/srv/_RotateAngle.py"
	/home/patrick/ros/ros/core/rospy/scripts/gensrv_py --noinitpy /home/patrick/Eclipse/serializer/src/ros/serializer/srv/RotateAngle.srv

ROSBUILD_gensrv_py: CMakeFiles/ROSBUILD_gensrv_py
ROSBUILD_gensrv_py: ../src/serializer/srv/__init__.py
ROSBUILD_gensrv_py: ../src/serializer/srv/_TravelDistance.py
ROSBUILD_gensrv_py: ../src/serializer/srv/_GetAnalog.py
ROSBUILD_gensrv_py: ../src/serializer/srv/_GetDigital.py
ROSBUILD_gensrv_py: ../src/serializer/srv/_SetDigital.py
ROSBUILD_gensrv_py: ../src/serializer/srv/_IsMoving.py
ROSBUILD_gensrv_py: ../src/serializer/srv/_RotateAngle.py
ROSBUILD_gensrv_py: CMakeFiles/ROSBUILD_gensrv_py.dir/build.make
.PHONY : ROSBUILD_gensrv_py

# Rule to build all files generated by this target.
CMakeFiles/ROSBUILD_gensrv_py.dir/build: ROSBUILD_gensrv_py
.PHONY : CMakeFiles/ROSBUILD_gensrv_py.dir/build

CMakeFiles/ROSBUILD_gensrv_py.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ROSBUILD_gensrv_py.dir/clean

CMakeFiles/ROSBUILD_gensrv_py.dir/depend:
	cd /home/patrick/Eclipse/serializer/src/ros/serializer/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/patrick/Eclipse/serializer/src/ros/serializer /home/patrick/Eclipse/serializer/src/ros/serializer /home/patrick/Eclipse/serializer/src/ros/serializer/build /home/patrick/Eclipse/serializer/src/ros/serializer/build /home/patrick/Eclipse/serializer/src/ros/serializer/build/CMakeFiles/ROSBUILD_gensrv_py.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ROSBUILD_gensrv_py.dir/depend

