# PCL Fusion

This is a software package developed as part of a larger collaborative SLAM framework, available [here](https://github.com/VIS4ROB-lab/multi_robot_coordination).  

The package receives raw pointclouds (e.g. from [image_undistort's dense_stereo node](https://github.com/ethz-asl/image_undistort)) along with keyframe messages from a [client-server adaptation of VINS-Mono](https://github.com/VIS4ROB-lab/vins_client_server/tree/feature/multi_agent). It fuses the pointclouds together to create a larger pointcloud with rigid body transformations obtained from the keyframe messages, and then downsamples it using a voxel filter. These are then sent to a backend server for further pose-graph optimization.

## Installation  
In order to install this package, follow these steps (tested under Ubuntu 18.04 LTS, ROS Melodic). First, create a catkin workspace:
```
$ mkdir -p catkin_ws/src
$ cd catkin_ws
```
Set-up the workspace:
```
$ catkin init
$ catkin config --extend /opt/ros/melodic
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
$ catkin config --merge-devel
```

Clone the dependencies:
```
$ cd ~/catkin_ws/src
$ wstool init
$ wstool merge pcl_fusion/dependencies_ssh.rosinstall # To clone with https: pcl_fusion/dependencies_https.rosinstall
$ wstool up -j8
```  

Finally, build the package:
```
$ cd ~/catkin_ws
$ catkin build pcl_fusion
```  

## Parameters:
* `voxel_filter_size`: The bigger the size, the more the pointcloud will be filtered, resulting in less resolution but lower size
* `num_odom_connections`: The number of sequential keyframes that should be linked for relative pose odometry optimzation.
* `agent_id`: The assigned ID of the agent that this is running on. Must be consistent across all packages.
