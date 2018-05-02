#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('team3_perception')
import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import numpy as np
from math import atan,tan,sqrt,sin,cos

class kalman_estimator:

  def __init__(self):
    self.seq = 0
    #self.image_sub = rospy.Subscriber("/bogey0/camera/fisheye/image_raw",Image,self.callback)
    #self.calib_sub = rospy.Subscriber("/bogey0/camera/fisheye/camera_info",CameraInfo,self.camInfo)
    self.state_sub = rospy.Subscriber("/bogey0/ball_pose",PointStamped,self.kalmanUpdate)
    self.new_state_pub = rospy.Publisher("/bogey0/clean_ball_pose",PointStamped,queue_size=10)
    self.logger = open('datalog_filter.csv','w')
    self.logger.write("x,y,z\n")
    self.xhat_old = np.array([[0],[0],[0],[0]])
    self.P_old = np.ones((4,4))

  def __del__(self):
    self.logger.close()

  def kalmanUpdate(self, raw_state):
    A = np.array([[ 0,1, 0,0],
                  [-1,0, 0,0],
                  [ 0,0, 0,1],
                  [ 0,0,-1,0]])

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
    
    print(measurement)
    print("\n")
    print(yhat)
    print("\n")
    print(xhat)
    print("\n")
    print(P)
    
    xhat_sq = xhat.squeeze()
    

    
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
