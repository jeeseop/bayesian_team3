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
import numpy
from math import atan,tan,sqrt,sin,cos
import math

_EPS = 0.00001

def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.

    >>> M = quaternion_matrix([0.99810947, 0.06146124, 0, 0])
    >>> numpy.allclose(M, rotation_matrix(0.123, [1, 0, 0]))
    True
    >>> M = quaternion_matrix([1, 0, 0, 0])
    >>> numpy.allclose(M, numpy.identity(4))
    True
    >>> M = quaternion_matrix([0, 1, 0, 0])
    >>> numpy.allclose(M, numpy.diag([1, -1, -1, 1]))
    True

    """
    q = numpy.array(quaternion, dtype=numpy.float64, copy=True)
    n = numpy.dot(q, q)
    if n < _EPS:
        return numpy.identity(4)
    q *= math.sqrt(2.0 / n)
    q = numpy.outer(q, q)
    return numpy.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])


def quaternion_inverse(quaternion):
    """Return inverse of quaternion.

    >>> q0 = random_quaternion()
    >>> q1 = quaternion_inverse(q0)
    >>> numpy.allclose(quaternion_multiply(q0, q1), [1, 0, 0, 0])
    True

    """
    q = numpy.array(quaternion, dtype=numpy.float64, copy=True)
    numpy.negative(q[1:], q[1:])
    return q / numpy.dot(q, q)


class kalman_estimator:

  def __init__(self):
    self.seq = 0
    #self.image_sub = rospy.Subscriber("/bogey0/camera/fisheye/image_raw",Image,self.callback)
    #self.calib_sub = rospy.Subscriber("/bogey0/camera/fisheye/camera_info",CameraInfo,self.camInfo)
    self.state_sub = rospy.Subscriber("/bogey0/ball_pose",PointStamped,self.kalmanUpdate)
    self.posit_sub = rospy.Subscriber("/gazebo/link_states",LinkStates,self.callback_pos)
    self.new_state_pub = rospy.Publisher("/bogey0/clean_ball_pose",PointStamped,queue_size=10)
    self.logger = open('datalog_filter.csv','w')
    self.logger.write("perc_error,kal_error,gt_bal_x,gt_bal_y,gt_bal_z,perc_bal_x,perc_bal_y,perc_bal_z,kal_bal_x,kal_bal_y,kal_bal_z\n")
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
    self.local_pos = data.pose[cam_idx]
    self.balloon_pos = data.pose[balloon_idx]
    
    #self.balloon_pos.position.z -= 2 # Rope Length #simulation fixed

  def kalmanUpdate(self, raw_state):
    A = np.array([[ 0,1, 0,0],
                  [-2*np.pi/3,0, 0,0],
                  [ 0,0, 0,1],
                  [ 0,0,-2*np.pi/3,0]])

    A= A + np.identity(4)

    C = np.array([[1,0,0,0],
                  [0,0,1,0]])
    # No Direct feedthrough
    
    Q = np.array([[0.02,0.02,   0,   0],
                  [0.02,0.02,   0,   0],
                  [   0,   0,0.02,0.02],
                  [   0,   0,0.02,0.02]]) #motion model covariance
                  
    R = np.array([[ 0.01, 0.001],
                  [0.001,  0.01]]) #sensor model covariance
    
    # PREDICTION ##############################################################
    xhat_k_km1 = A.dot(self.xhat_old) # Ignoring B term
    P_k_km1 = A.dot(self.P_old).dot(A.T) + Q
    
    # CORRECTION ##############################################################
    
    # Z values are thrown out
    #measurement = np.array([[raw_state.point.x],
    #                        [raw_state.point.y]])
    
    measurement = np.array([[raw_state.point.y],
                            [raw_state.point.x]])

    K = P_k_km1.dot(C.T).dot(np.linalg.inv(C.dot(P_k_km1).dot(C.T) + R))
    
    xhat = xhat_k_km1 + K.dot(measurement - C.dot(xhat_k_km1))
    P = (np.eye(4) - K.dot(C)).dot(P_k_km1)

    yhat = C.dot(xhat)
    
    yhat_sq = yhat.squeeze()
    
    #print(measurement)
    #print("\n")
    #print(yhat)
    #print("\n")
    #print(xhat)
    #print("\n")
    #print(P)
    
    xhat_sq = xhat.squeeze()
    
    if self.local_pos is not None:  
    
      balloon_meas = np.array([[yhat_sq[0]],[yhat_sq[1]],[raw_state.point.z],[1]])
      
      print("\n\n KALMAN BALLOON DATA \n\n")
      
      print(balloon_meas)
      
      local_bogey_quat = np.array([self.local_pos.orientation.w,self.local_pos.orientation.x,self.local_pos.orientation.y,self.local_pos.orientation.z])
      
      #print(local_bogey_quat)
      
      bogey_rotmat = quaternion_matrix((local_bogey_quat)) # quaternion_inverse
      
      #print(bogey_rotmat)
    
      est_global_pos = bogey_rotmat.dot(balloon_meas)
      
      #est_global_pos = est_global_pos[:3,:]
      #est_global_pos = np.array([[-est_global_pos[1,0]],
      #                           [est_global_pos[2,0]],
      #                           [est_global_pos[0,0]]])
      
      est_global_pos = np.array([[yhat_sq[0]],
                                 [yhat_sq[1]],
                                 [-raw_state.point.z]])#kalman filtered
      
      est_raw_pos = np.array([[ raw_state.point.y],
                              [ raw_state.point.x],
                              [-raw_state.point.z]])#only perception
      
      print("\n\nWORLD ALIGNED MEASUREMENT\n\n")
      
      print(est_global_pos)
      
      est_global_pos = est_global_pos + \
                       np.array([[self.local_pos.position.x],
                                 [self.local_pos.position.y],
                                 [self.local_pos.position.z]])
                                 
      est_raw_pos = est_raw_pos + \
                    np.array([[self.local_pos.position.x],
                              [self.local_pos.position.y],
                              [self.local_pos.position.z]])
          
      print("\n\nBOGEY0 POS:\n\n")
      print(self.local_pos.position)
      print("\n\nPERCEPTED BALLOON POS:\n\n")
      print(measurement)
      print("\n\nEST GLOBAL BALLOON POS: \n\n")             
      print(est_global_pos)
      print("\n\nREAL GLOBAL BALLOON POS: \n\n")
      print(self.balloon_pos.position)
    
      mse_raw = np.linalg.norm(est_raw_pos - np.array([[self.balloon_pos.position.x],
                              [self.balloon_pos.position.y],
                              [self.balloon_pos.position.z]]))
    
      mse_kal = np.linalg.norm(est_global_pos - np.array([[self.balloon_pos.position.x],
                              [self.balloon_pos.position.y],
                              [self.balloon_pos.position.z]]))
    
      print("\n\nERROR_RAW: {}".format(mse_raw))
      print("ERROR_KALMAN: {}\n\n".format(mse_kal))
    
      self.logger.write("{},{},{},{},{},{},{},{},{},{},{}\n".format(
                        mse_raw,mse_kal,
			                  self.balloon_pos.position.x,self.balloon_pos.position.y,self.balloon_pos.position.z,
                        est_raw_pos[0,0],est_raw_pos[1,0],est_raw_pos[2,0],
                        est_global_pos[0,0],est_global_pos[1,0],est_global_pos[2,0]))
    
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
