# CHANGELOG

## [galactic-0.0.1] - 2022-07-12
 
- Created Repository
- Initial Batch
 
### New features

- Linux Container which includes [airsim_ros_pkgs](https://github.com/microsoft/AirSim/tree/main/ros2/src/airsim_ros_pkgs)
- Includes minimum components to run ROS2 node
    - Only includes built `AirLib`, `MavLinkCom`, `cmake`, `external`, `ros2` from [AirSim](https://github.com/microsoft/AirSim)
- Targets ROS2 node container to subscribe/publish AirSim msgs from local/external AirSim instance
 
### Modifications