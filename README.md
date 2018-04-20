# bayesian_team3


## target specification

* UAV with radius 0.25m sphere

* Red color



## start gazebo simulation environment
```
roslaunch mbzirc_gazebo challenge1_simulator_proj2.launch
```

## start uav operating software
```
roslaunch mbzirc_gazebo challenge1_system_proj2.launch
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
