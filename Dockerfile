FROM amd64/ros:galactic-ros-base

# ---------- SET BASIC SETTINGS ----------
ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV simhost localhost
 
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

WORKDIR /root

RUN git clone https://github.com/microsoft/AirSim.git -b v1.8.0-linux \
	&& ./AirSim/setup.sh \
	&& ./AirSim/build.sh \
	&& chmod +x ./AirSim/tools/install_ros2_deps.sh

RUN apt update && apt install -y \
	ros-$ROS_DISTRO-vision-opencv \
	ros-${ROS_DISTRO}-image-transport \
	libyaml-cpp-dev \
	ros-${ROS_DISTRO}-tf2-sensor-msgs \
	ros-${ROS_DISTRO}-tf2-geometry-msgs \
	ros-${ROS_DISTRO}-mavros*

WORKDIR /root/AirSim/ros2

RUN source /opt/ros/galactic/setup.bash \
	&& echo 'source /root/AirSim/ros2/install/setup.bash' >> /root/.bashrc \
	&& colcon build \
		--cmake-args -DCMAKE_C_COMPILER=gcc-8 \
		--cmake-args -DCMAKE_CXX_COMPILER=g++-8 \
		--cmake-args -DCMAKE_BUILD_TYPE=Debug; exit 0

#COPY AirSim/entrypoint.sh /usr/local/bin/entrypoint.sh
# CMD [ "/usr/local/bin/entrypoint.sh" ]
CMD ["bash", "-ic", "ros2 launch airsim_ros_pkgs airsim_node.launch.py host:=$simhost"]