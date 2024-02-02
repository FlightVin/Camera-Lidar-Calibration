# ROS Simulations

## Contents

### Directories

| S. No. | File/Directory Names | Description |
| :----- | :--------- | :------ |
| 1 | turtle_bot_sim_using_gazebo.md | Instructions to run the simulations for calibration
| 2 | catkin_ws | Catkin Workspace that is built for ROS
| 3 | data_collection | Scripts for subscribing to lidar and camera data. Data collected during experiments. Scripts to visualize collected data.
| 4 | my-checkerboard-gen.py | Script to make custom checkerboard models
| 5 | basic_ros_running_instructions.md | Instructions written to understand fundamental ROS usage

### Catkin WS

Packages in source are:

1. realsense_gazebo_plugin: Plugin for using realsense camera in ROS
2. realsense2_description: Models for realsense camera
3. turtlebot3_simulations: Models and launch files for turtle bot

### Data Collection

Contents are:

1. 2d: folder storing data of 2D experiments
2. 2d_calib_image_sub.py : Subscriber for image data of 2D experiments
3. 2d_lidar_subscriber.py : Subcriber for lidar data of 2D experiments
4. 2d_pointcloud_viz.py : Visualizer of pointloud (bag files) of 2D experiments
5. 3d : folder storing data of 3D experiments
6. 3d_lidar_cam_calib_subscriber.py : Deprecated
7. 3d_lidar_subcriber.py : Subcriber foor lidar data of 3D experiments
8. 3d_pointcloud_viz.py : Visualizer of pointloud (bag files) of 3D experiments
9. image_subscriber.py : Subscriber for image data of 3D experiments
