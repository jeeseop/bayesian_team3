#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('team3_perception')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import PointStamped
from gazebo_msgs.msg import LinkStates
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from math import atan,tan,sqrt,sin,cos

# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

class image_converter:

  def __init__(self):

    #self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.seq = 0
    self.image_sub = rospy.Subscriber("/bogey0/camera/fisheye/image_raw",Image,self.callback)
    self.calib_sub = rospy.Subscriber("/bogey0/camera/fisheye/camera_info",CameraInfo,self.camInfo)
    self.state_pub = rospy.Publisher("/bogey0/ball_pose",PointStamped,queue_size=10)
    self.posit_sub = rospy.Subscriber("/gazebo/link_states",LinkStates,self.callback_pos)
    self.cameraMat = None
    self.logger = open('datalog.csv','w')
    self.logger.write("x,y,z\n")
    
    self.balloon_pose = None
    self.local_pose = None

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

  def camInfo(self,data):
    self.cameraMat = np.array(data.K).reshape((3,3))
    self.distCoefs = np.array(data.D).reshape((5,))

  def callback(self,data):
    #print(dir(data))
    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #frame = frame[:,:,2] # Select red channel
    colors = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    # HSV
    #redish = cv2.inRange(colors,(180,200,0),(255,255,80))
    redish = cv2.inRange(frame,(0,0,200),(55,55,255))

    redish = cv2.morphologyEx(redish,cv2.MORPH_OPEN,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5)))

    im2,contours,hierarchy = cv2.findContours(redish, 1, 2)
    #print(contours[0])
    
    center = None
    try:
        contours_undist = contours[0].reshape((1,-1,2)).astype(np.float32)
        #print(contours_undist)
        #contours_undist = cv2.fisheye.undistortPoints(contours_undist,self.cameraMat,np.zeros((4,)),None,None)
        #print(contours_undist)
        cnt = contours[0]
        M = cv2.moments(cnt)

        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        (x,y),radius = cv2.minEnclosingCircle(cnt)
        center = (int(x),int(y))
        #radius = int(radius
        cv2.circle(frame,center,int(radius),(0,255,0),2)

        cv2.circle(frame,(int(cx),int(cy)),5,(255,0,0),-1)
    except IndexError:
        pass
    
    frame[frame.shape[0]/2,:,:] = 255
    frame[:,frame.shape[1]/2,:] = 255

    #cv2.imshow("HSV", colors)
    cv2.imshow("Image window", frame)
    cv2.imshow("Threshold", redish)
    cv2.waitKey(1)

    output = PointStamped()

    output.header.seq = self.seq
    output.header.stamp = rospy.Time.now()
    output.header.frame_id = "bogey0/fisheye_camera_optical_frame"

    if self.cameraMat is not None and center is not None:
        #pts = np.array([[[center[0],center[1]]]]).astype(np.float32)
        #print(pts.shape)
        #pts = cv2.fisheye.undistortPoints(pts,self.cameraMat,np.zeros((4,)),None,None).squeeze()/1000 # [m]
        #print(pts)
        fov_x = 2*atan(frame.shape[1]/(2*self.cameraMat[0,0]))
        fov_y = fov_x*frame.shape[0]/frame.shape[1]
        #theta = radius/float(frame.shape[1])/fov_x
        theta = 2*radius/frame.shape[0]*fov_y
        theta = theta*1.68 # Arbitrary scaling factor
        print("FOV: {},{}".format(fov_x,fov_y))
        print("THETA: "+str(theta))
        distance = 0.25/tan(theta/2.0) # [m]
        #distance = distance * 0.045 # Arbitrary scaling factor
        print("DISTANCE: "+str(distance))

        theta = (center[0]-self.cameraMat[0,2])/frame.shape[1]*fov_x
        phi   = (center[1]-self.cameraMat[1,2])/frame.shape[0]*fov_y
        print(theta,phi)
        output.point.x = sin(theta)*distance
        output.point.y = sin(phi)*distance
        output.point.z = sqrt(distance*distance - output.point.x*output.point.x - output.point.y*output.point.y)

        print(output.point)

        if self.balloon_pos is not None:
          x = self.balloon_pos.position.x - self.local_pos.position.x
          y = self.balloon_pos.position.y - self.local_pos.position.y
          z = self.balloon_pos.position.z - self.local_pos.position.z
          real_dist = sqrt(x*x+y*y+z*z)
          print("REAL DIST: {}".format(real_dist))
        self.logger.write("{},{},{}\n".format(output.point.x,output.point.y,output.point.z))
        self.state_pub.publish(output)

    self.seq += 1

def main(args):
  ic = image_converter()
  rospy.init_node('team3_perception', anonymous=True)
  try:
    rospy.spin()
  except LeopardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
