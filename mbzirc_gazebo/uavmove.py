#! /usr/bin/python

import subprocess
import time
import random

#cmd="roslaunch mbzirc_gazebo challenge1_simulator_proj1.launch"

x=1
cycle_num=10
time_gap=5

moving_z=5.0
moving_y=5.0
moving_x=5.0


takeoff="rosservice call /bogey1/offboard_control/takeoff '{}'"
takeoff_value=subprocess.call(takeoff, shell=True)
time.sleep(5)



cmd1="rosservice call /bogey1/offboard_control/waypoint '{ position: { x: 0.0, y: " + str(moving_y) + ", z: " + str(moving_z) + "}, yaw: 0.0 }'"
cmd1_value=subprocess.call(cmd1, shell=True)
time.sleep(5)


while True:
	
	print x
	
	moving_z=random.uniform(1.0,5.0)
	moving_y=random.uniform(1.0,5.0)
	moving_x=random.uniform(2.5,8.0)

	cmd2="rosservice call /bogey1/offboard_control/waypoint '{ position: { x: " + str(moving_x) + ", y: " + str(moving_y) + ", z: " + str(moving_z) + " }, yaw: 0.0 }'"
	cmd2_value=subprocess.call(cmd2, shell=True)
	time.sleep(time_gap)

	cmd3="rosservice call /bogey1/offboard_control/waypoint '{ position: { x: " + str(moving_x) + ", y: -" + str(moving_y) + ", z: " + str(moving_z) + " }, yaw: 0.0 }'"
	cmd3_value=subprocess.call(cmd3, shell=True)
	time.sleep(time_gap)

	cmd4="rosservice call /bogey1/offboard_control/waypoint '{ position: { x: -" + str(moving_x) + ", y: -" + str(moving_y) + ", z: " + str(moving_z) + " }, yaw: 0.0 }'"
	cmd4_value=subprocess.call(cmd4, shell=True)
	time.sleep(time_gap)

	cmd5="rosservice call /bogey1/offboard_control/waypoint '{ position: { x: -" + str(moving_x) + ", y: " + str(moving_y) + ", z: " + str(moving_z) + " }, yaw: 0.0 }'"
	cmd5_value=subprocess.call(cmd5, shell=True)
	time.sleep(time_gap)
	
	x+=1
	
	if x==cycle_num:
		break


cmd6="rosservice call /bogey1/offboard_control/waypoint '{ position: { x: 0.0, y: 0.0, z: 5.0 }, yaw: 0.0 }'"
cmd5_value=subprocess.call(cmd6, shell=True)
time.sleep(time_gap)
