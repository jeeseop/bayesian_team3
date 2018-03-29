# bayesian_team3


## target specification

* radius 0.5m sphere

* Red color

* starting pos. : 20m, 20m, 2m



## start gazebo simulation environment
```
roslaunch mbzirc_gazebo challenge1_simulator_proj1.launch
```

## start uav operating software
```
roslaunch mbzirc_gazebo challenge1_system_proj1.launch
```



## manipulate target motion using wrench
* This command is appling force, thus you have to command reverse force if you want to move target back to the field of view
* I will try to make the command more automatic
* force value can change, and also it can move upward/downward


1. apply force
```
rosservice call /gazebo/apply_body_wrench '{body_name: "balloon::link", reference_frame: "balloon::link", wrench: { force: { x: -0.5, y: -0.5, z: 0 } }, start_time: 0, duration: -1 }'
```

2. stop appling force
```
roslaunch mbzirc_gazebo balloonstop.launch
```



