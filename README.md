# PCL Fusion

This is a software package developed as part of a larger collaborative SLAM framework, available [here](https://github.com/VIS4ROB-lab/multi_robot_coordination).  

The package receives raw pointclouds (e.g. from [image_undistort's dense_stereo node](https://github.com/ethz-asl/image_undistort)) along with keyframe messages from a [client-server adaptation of VINS-Mono](https://github.com/VIS4ROB-lab/vins_client_server/tree/feature/multi_agent). It fuses the pointclouds together to create a larger pointcloud with rigid body transformations obtained from the keyframe messages, and then downsamples it using a voxel filter. These are then sent to a backend server for further pose-graph optimization.

## Parameters:
* `voxel_filter_size`: The bigger the size, the more the pointcloud will be filtered, resulting in less resolution but lower size
* `num_odom_connections`: The number of sequential keyframes that should be linked for relative pose odometry optimzation.
* `agent_id`: The assigned ID of the agent that this is running on. Must be consistent across all packages.
