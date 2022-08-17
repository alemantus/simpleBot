#!/usr/bin/env python

"""follower_ros.py: Robot will follow the Yellow Line in a track"""

__author__  = "Arjun S Kumar"

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('/main_camera/image_raw', Image, self.image_callback)
    #self.image_comp_sub = rospy.Subscriber('/main_camera/image_raw/compressed', CompressedImage, self.image_callback)


    self.image_comp_pub = rospy.Publisher('/main_camera/image_raw/features/compressed',CompressedImage, queue_size = 1)
    self.mask_comp_pub = rospy.Publisher('/main_camera/image_raw/mask/compressed',CompressedImage, queue_size = 1)
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.twist = Twist()

    self.upperThreshold = rospy.set_param('image_proc/upperThreshold', 255)
    self.lowerThreshold = rospy.set_param('/image_proc/lowerThreshold', 150)
    self.lowerThreshold = rospy.set_param('/image_proc/seachArea', 20)

  def image_callback(self, msg):

    self.upperThreshold = rospy.get_param('image_proc/upperThreshold')
    self.lowerThreshold = rospy.get_param('image_proc/lowerThreshold')
    self.searchArea = rospy.get_param('/image_proc/seachArea')

    self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')


    h, w, d = self.image.shape
    ROI = self.image[h/3:h/3+h, 0:w]

    img_gray = cv2.cvtColor(ROI, cv2.COLOR_BGR2GRAY)
    img_gray = cv2.GaussianBlur(img_gray, (7, 7), 0)
    (T, mask) = cv2.threshold(img_gray, self.lowerThreshold, self.upperThreshold,cv2.THRESH_BINARY)

    search_top = h/4
    search_bot = h/4+20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0


    contours,hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    numberOfContours = len(contours)
    x = [None]*len(contours)
    y = [None]*len(contours)
    w = [None]*len(contours)
    h = [None]*len(contours)
    a = [None]*len(contours)

    
    path="left"
    for i,c in enumerate(contours):
      x[i], y[i], w[i], h[i] = cv2.boundingRect(c)
      
      a[i] = w[i]*h[i]

    # If there are more than 1 contour and the smallest one has a large area
    if (len(contours) > 1 and np.min(a)>600):
      rospy.logdebug("Crossdetected")
    

      if path == "left":
        index = np.argmin(x)

        cv2.rectangle(ROI, (x[index],y[index]), (x[index]+w[index], y[index]+h[index]), (0, 0, 255), 2)
        cv2.circle(ROI, (x[index]+w[index]/2, y[index]+h[index]/2), 20, (0,0,255), -1)
        
      if path == "right":
        index = np.argmax(x)
        cv2.rectangle(ROI, (x[index],y[index]), (x[index]+w[index], y[index]+h[index]), (0, 0, 255), 2)
        cv2.circle(ROI, (x[index]+w[index]/2, y[index]+h[index]/2), 20, (0,0,255), -1)

    else:
      rospy.logdebug("single line")
      cv2.rectangle(ROI, (x[0],y[0]), (x[0]+w[0], y[0]+h[0]), (0, 0, 255), 2)
      cv2.circle(ROI, (x[0]+w[0]/2, y[0]+h[0]/2), 20, (0,0,255), -1)


    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', ROI)[1]).tostring()

    mask_pub = CompressedImage()
    mask_pub.header.stamp = rospy.Time.now()
    mask_pub.format = "jpeg"
    mask_pub.data = np.array(cv2.imencode('.jpg', mask)[1]).tostring()


    self.image_comp_pub.publish(msg)
    self.mask_comp_pub.publish(mask_pub)
      # CONTROL ends


rospy.init_node('line_follower', log_level=rospy.DEBUG)
follower = Follower()
rospy.spin()
# END ALL