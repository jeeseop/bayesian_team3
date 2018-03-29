# bayesian_team3


target specification

radius 0.5m sphere

Red color



start gazebo simulation environment
roslaunch mbzirc_gazebo challenge1_simulator_proj1.launch

start uav operating software
roslaunch mbzirc_gazebo challenge1_system_proj1.launch


manipulate target motion using wrench

apply force
rosservice call /gazebo/apply_body_wrench '{body_name: "balloon::link", reference_frame: "balloon::link", wrench: { force: { x: 0, y: 1, z: 0 } }, start_time: 0, duration: -1 }'

stop appling force
rosservice call /gazebo/clear_body_wrenches '{body_name: "balloon::link"}'




