ARG BASEIMAGE=amd64/ros
ARG BASETAG=galactic-ros-base


FROM ${BASEIMAGE}:${BASETAG} as stage_apt

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN rm -f /etc/apt/apt.conf.d/docker-clean \
	&& echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache \
	&& apt-get update


FROM ${BASEIMAGE}:${BASETAG} as stage_airsim

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV \
	DEBIAN_FRONTEND=noninteractive \
	LANG=C.UTF-8 \
	LC_ALL=C.UTF-8 \
	simhost=localhost \
	rebuild=false

RUN git clone https://github.com/microsoft/AirSim.git -b v1.8.0-linux /root/AirSim \
	&& /root/AirSim/setup.sh \
	&& /root/AirSim/build.sh

RUN \
    --mount=type=cache,target=/var/cache/apt,from=stage_apt,source=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt,from=stage_apt,source=/var/lib/apt \
	apt-get install --no-install-recommends -y \
		ros-$ROS_DISTRO-vision-opencv \
		ros-${ROS_DISTRO}-image-transport \
		libyaml-cpp-dev \
		ros-${ROS_DISTRO}-tf2-sensor-msgs \
		ros-${ROS_DISTRO}-tf2-geometry-msgs \
		ros-${ROS_DISTRO}-mavros* \
		python3-pip \
		python3-yaml \
		python3-setuptools \
		python3-colcon-common-extensions

RUN \
	pip install opencv-python msgpack-rpc-python flask numpy \
	&& pip install airsim

WORKDIR /root/AirSim/ros2

RUN source /opt/ros/galactic/setup.bash \
	&& echo 'source /opt/ros/galactic/setup.bash' >> /root/.bashrc \
	&& colcon build \
		--cmake-args -DCMAKE_C_COMPILER=gcc-8 \
		--cmake-args -DCMAKE_CXX_COMPILER=g++-8 \
		--cmake-args -DCMAKE_BUILD_TYPE=Debug; exit 0

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
CMD [ "/usr/local/bin/entrypoint.sh" ]
# CMD ["bash", "-ic", "ros2 launch airsim_ros_pkgs airsim_node.launch.py host:=$simhost"]