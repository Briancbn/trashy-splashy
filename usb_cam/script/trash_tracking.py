#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from collections import deque
import numpy as np
import imutils


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/trash_tracking",Image, queue_size = 1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    # construct the argument parse and parse the command line arguments

    # define the lower and upper HSV (hue, saturation, value) boundaries of the orange
    # ball in the HSV color space, then initialize the
    # list of tracked points
    # use the colortrack to determine the HSV value
    self.OrangeLower = (0, 41,132)
    self.OrangeUpper = (21, 232, 255)
    self.pts = deque(maxlen=64)
    #deque is a list-like data structure that maintains
    #past N (x, y)-locations of the ball in our video stream which
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # resize the frame, blur it, and convert it to the HSV
    # color space
    cv_image = imutils.resize(cv_image, width=600)
    # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, self.OrangeLower, self.OrangeUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    res = cv2.bitwise_and(cv_image,cv_image, mask= mask)



    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    	cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
    	# find the largest contour in the mask, then use
    	# it to compute the minimum enclosing circle and
    	# centroid
    	c = max(cnts, key=cv2.contourArea)
    	((x, y), radius) = cv2.minEnclosingCircle(c)
    	M = cv2.moments(c)
    	center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

    	# only proceed if the radius meets a minimum size
    	if radius > 10:
    		# draw the circle and centroid on the frame,
    		# then update the list of tracked points
    		cv2.circle(cv_image, (int(x), int(y)), int(radius),
    			(0, 255, 255), 2)
    		cv2.circle(cv_image, center, 5, (0, 0, 255), -1)

    # update the points queue
    self.pts.appendleft(center)

    for i in range(1, len(self.pts)):
    	# if either of the tracked points are None, ignore
    	# them
    	if self.pts[i - 1] is None or self.pts[i] is None:
    		continue

    	# otherwise, compute the thickness of the line and
    	# draw the connecting lines
    	thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
    	cv2.line(cv_image, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)

    # show the frame to our screen


    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
