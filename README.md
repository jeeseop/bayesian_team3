# bayesian_team3


## target specification

* radius 0.5m sphere

* mass 1kg (use this info. for choosing the applied force)

* Red color

* starting pos. : 20m, 20m, 2m (you can manage these values in challenge1_simulator_proj1.launch)



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
* force value can change, and also if you choose proper force, target will clime


1. apply force
```
rosservice call /gazebo/apply_body_wrench '{body_name: "balloon::link", reference_frame: "balloon::link", wrench: { force: { x: -0.5, y: -0.5, z: 0 } }, start_time: 0, duration: -1 }'
```

2. stop appling force
```
roslaunch mbzirc_gazebo balloonstop.launch
```



## Perception

Make sure you've launched the UAV in the air, so that it can see the red ball.


You may also need to move it near the ball.


*Note*: The Z estimate is very much an estimate, there's something I'm not understanding
about the perspective math. I've applied a scaling factor which shouldn't be necessary to make the 
numbers reasonable. Later I will update it.

```
rosrun team3_perception perception.py
```
