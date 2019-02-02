# path_prediction
In this repository, I compiled the source code using ROS for path prediction of pedestrians

<!-- ## Usage -->
#### Requirement
- normal_reaction_force (https://github.com/matchey/normal_reaction_force)
- kf_tracking (https://github.com/matchey/kf_tracking)
- perfect_velodyne (https://github.com/matchey/perfect_velodyne)
- human_detection (https://github.com/matchey/human_detection)
- complement (https://github.com/matchey/complement)
- mmath (https://github.com/matchey/mmath)

#### How to run
```
$ roslaunch perfect_velodyne 32e_points.launch
$ rosrun perfect_velodyne rm_ground
$ roslaunch complement complement_pubtf.launch
$ roslaunch human_detection human_pos_publisher.launch
$ roslaunch kf_tracking human_tracking.launch
$ rosrun path_prediction test_path_prediction
```

<!-- rosparam set /use_sim_time true -->
<!-- rosbag play xxxx-xx-xx --clock -->
<!-- rviz -d .rviz/path_prediction.rviz -->

