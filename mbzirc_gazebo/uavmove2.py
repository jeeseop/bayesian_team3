#! /usr/bin/python

import subprocess
import time
import random
import math

#cmd="roslaunch mbzirc_gazebo challenge1_simulator_proj1.launch"

x=0
cycle_num=2
time_gap=0.1
period=3

x_axis=4
y_axis=3

moving_z=5.0
moving_y=0.0
moving_x=2.0

######## take off uav
takeoff0="rosservice call /bogey0/offboard_control/takeoff '{}'"
takeoff_value=subprocess.call(takeoff0, shell=True)
time.sleep(5)


cmd0="rosservice call /bogey0/offboard_control/waypoint '{ position: { x: 0.0, y: 0.0, z: 7.0}, yaw: 0.0 }'"
cmd1_value=subprocess.call(cmd0, shell=True)
time.sleep(5)


takeoff="rosservice call /bogey1/offboard_control/takeoff '{}'"
takeoff_value=subprocess.call(takeoff, shell=True)
time.sleep(5)


cmd1="rosservice call /bogey1/offboard_control/waypoint '{ position: { x: 0.0, y: " + str(moving_y) + ", z: " + str(moving_z) + "}, yaw: 0.0 }'"
cmd1_value=subprocess.call(cmd1, shell=True)
time.sleep(5)


while True:
	
	print x
	
#	moving_x=random.uniform(1.0,5.0)
#	moving_y=random.uniform(1.0,5.0)
#	moving_z=random.uniform(2.5,8.0)

        moving_x=x_axis*math.cos(2*3.141592*x*time_gap/period)-x_axis
        moving_y=y_axis*math.sin(2*3.141592*x*time_gap/period)
        moving_z=5.0

	cmd2="rosservice call /bogey1/offboard_control/waypoint '{ position: { x: " + str(moving_x) + ", y: " + str(moving_y) + ", z: " + str(moving_z) + " }, yaw: 0.0 }'"
	cmd2_value=subprocess.call(cmd2, shell=True)
	time.sleep(time_gap)

	
	x+=1
	
	if x==cycle_num*period/time_gap:
		break


cmd6="rosservice call /bogey1/offboard_control/waypoint '{ position: { x: 0.0, y: 0.0, z: 5.0 }, yaw: 0.0 }'"
cmd5_value=subprocess.call(cmd6, shell=True)
time.sleep(time_gap)
