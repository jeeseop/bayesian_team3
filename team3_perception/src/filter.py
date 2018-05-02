#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('team3_perception')
import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from gazebo_msgs.msg import LinkStates
import numpy as np
from math import atan,tan,sqrt,sin,cos

class kalman_estimator:

  def __init__(self):
    self.seq = 0
    #self.image_sub = rospy.Subscriber("/bogey0/camera/fisheye/image_raw",Image,self.callback)
    #self.calib_sub = rospy.Subscriber("/bogey0/camera/fisheye/camera_info",CameraInfo,self.camInfo)
    self.state_sub = rospy.Subscriber("/bogey0/ball_pose",PointStamped,self.kalmanUpdate)
    self.posit_sub = rospy.Subscriber("/gazebo/link_states",LinkStates,self.callback_pos)
    self.new_state_pub = rospy.Publisher("/bogey0/clean_ball_pose",PointStamped,queue_size=10)
    self.logger = open('datalog_filter.csv','w')
    self.logger.write("x,y,z\n")
    self.xhat_old = np.array([[0],[0],[0],[0]])
    self.P_old = np.ones((4,4))
    self.local_pos = None
    self.balloon_pos = None

  def __del__(self):
    self.logger.close()
    
  def callback_pos(self,data):
    #print(data.name)
    balloon_idx = data.name.index('bogey1::balloon_link')
    #print(balloon_idx)
    #print(data.pose[balloon_idx].position)
    cam_idx = data.name.index('bogey0::fisheye_camera')
    #print(cam_idx)
    self.local_pos = data.pose[cam_idx].position
    self.balloon_pos = data.pose[balloon_idx].position

  def kalmanUpdate(self, raw_state):
    A = np.array([[ 0,1, 0,0],
                  [-2*np.pi/3,0, 0,0],
                  [ 0,0, 0,1],
                  [ 0,0,-2*np.pi/3,0]])

    C = np.array([[1,0,0,0],
                  [0,0,1,0]])
    # No Direct feedthrough
    
    Q = np.array([[0.02,0.02,   0,   0],
                  [0.02,0.02,   0,   0],
                  [   0,   0,0.02,0.02],
                  [   0,   0,0.02,0.02]])
                  
    R = np.array([[ 0.01, 0.001],
                  [0.001,  0.01]])
    
    # PREDICTION ##############################################################
    xhat_k_km1 = A.dot(self.xhat_old) # Ignoring B term
    P_k_km1 = A.dot(self.P_old).dot(A.T) + Q
    
    # CORRECTION ##############################################################
    
    # Z values are thrown out
    measurement = np.array([[raw_state.point.x],
                            [raw_state.point.y]])
    
    K = P_k_km1.dot(C.T).dot(np.linalg.inv(C.dot(P_k_km1).dot(C.T) + R))
    
    xhat = xhat_k_km1 + K.dot(measurement - C.dot(xhat_k_km1))
    P = (np.eye(4) - K.dot(C)).dot(P_k_km1)

    yhat = C.dot(xhat)
    
    yhat_sq = yhat.squeeze()
    
    print(measurement)
    print("\n")
    print(yhat)
    print("\n")
    print(xhat)
    print("\n")
    print(P)
    
    xhat_sq = xhat.squeeze()
    
    if self.local_pos is not None:  
    
      est_global_pos = np.array([-yhat_sq[0],-yhat_sq[1],-raw_state.point.z])
      
      est_global_pos = est_global_pos + \
                       np.array([self.local_pos.x,
                                 self.local_pos.y,
                                 self.local_pos.z])
          
      print("EST GLOBAL POS: \n\n")             
      print(est_global_pos)
      print("\n\nREAL GLOBAL POS: \n\n")
      print(self.balloon_pos)
    
      self.logger.write("{},{},{}\n".format(xhat_sq[0],xhat_sq[1],xhat_sq[2]))
    
    output = PointStamped()

    output.header.seq = self.seq
    output.header.stamp = rospy.Time.now()
    output.header.frame_id = "bogey0/fisheye_camera_optical_frame"
    
    output.point.x = yhat[0]
    output.point.y = yhat[1]
    output.point.z = raw_state.point.z
    
    self.new_state_pub.publish(output)
    
    self.xhat_old = xhat
    self.P_old = P
    
    self.seq += 1
    
  def callback(self,data):
    pass

def main(args):
  k_e = kalman_estimator()
  rospy.init_node('team3_filter', anonymous=True)
  try:
    rospy.spin()
  except LeopardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
