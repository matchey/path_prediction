# path_prediction
In this repository, I compiled the source code using ROS for path prediction of pedestrians

<!-- ## Usage -->
#### Requirement
- normal_reaction_force (https://github.com/matchey/normal_reaction_force)
- kf_tracking (https://github.com/matchey/kf_tracking)
- complement (https://github.com/matchey/complement)
- mmath (https://github.com/matchey/mmath)

#### How to run
```
$ rosrun path_prediction test_path_prediction
$ roslaunch kf_tracking human_tracking.launch
$ roslaunch complement complement_pubtf.launch
```

