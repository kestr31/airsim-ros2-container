#!/bin/bash

if [ -n "${ROS_DISTRO}" ]; then
	source "/opt/ros/$ROS_DISTRO/setup.bash"
fi

if ${rebuild}; then
	colcon build \
		--cmake-args -DCMAKE_C_COMPILER=gcc-8 \
		--cmake-args -DCMAKE_CXX_COMPILER=g++-8 \
		--cmake-args -DCMAKE_BUILD_TYPE=Debug; exit 0
fi

source /root/AirSim/ros2/install/setup.bash
ros2 launch airsim_ros_pkgs airsim_node.launch.py host:=$simhost