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
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from math import atan,tan

# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

class image_converter:

  def __init__(self):

    #self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.seq = 0
    self.image_sub = rospy.Subscriber("/bogey0/camera/fisheye/image_raw",Image,self.callback)
    self.calib_sub = rospy.Subscriber("/bogey0/camera/fisheye/camera_info",CameraInfo,self.camInfo)
    self.state_pub = rospy.Publisher("/bogey0/ball_pose",PointStamped,queue_size=10)
    self.cameraMat = None

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
    center = None
    try:
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
    
    #cv2.imshow("HSV", colors)
    cv2.imshow("Image window", frame)
    cv2.imshow("Threshold", redish)
    cv2.waitKey(1)

    output = PointStamped()

    output.header.seq = self.seq
    output.header.stamp = rospy.Time.now()
    output.header.frame_id = "bogey0/fisheye_camera_optical_frame"

    if self.cameraMat is not None and center is not None:
        pts = np.array([[[center[0],center[1]]]]).astype(np.float32)
        print(pts.shape)
        pts = cv2.undistortPoints(pts,self.cameraMat,self.distCoefs,None,None).squeeze()/1000 # [m]
        print(pts)
        fov = 2*atan(frame.shape[1]/(2*self.cameraMat[0,0]))
        theta = radius/float(frame.shape[1])/fov
        print("FOV: "+str(fov))
        print("THETA: "+str(theta))
        distance = 0.5/tan(theta/2.0) # [m]
        distance = distance * 0.04 # Arbitrary scaling factor
        print("DISTANCE: "+str(distance))

        output.point.x = pts[0]
        output.point.y = pts[1]
        output.point.z = distance

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
