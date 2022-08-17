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

    #image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE) 
    #hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
    # change below lines to map the color you wanted robot to follow
    #lower_yellow = numpy.array([ 10,  10,  10])
    #upper_yellow = numpy.array([255, 255, 250])
    #mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    h, w, d = self.image.shape
    ROI = self.image[h/3:h/3+h, 0:w]

    img_gray = cv2.cvtColor(ROI, cv2.COLOR_BGR2GRAY)
    img_gray = cv2.GaussianBlur(img_gray, (7, 7), 0)
    (T, mask) = cv2.threshold(img_gray, self.lowerThreshold, self.upperThreshold,cv2.THRESH_BINARY)
    fullMask = mask.copy()

    search_top = h/4
    search_bot = h/4+20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    

    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      #cy = int(M['m00']/M['m01'])
      cy = int(M['m01']/M['m00'])
      h, w, d = ROI.shape
      
      
      
      #cv2.circle(ROI, (cx, cy), 20, (0,0,255), -1)

      path = 0
      
      if(path == 1):
        mask_right =  mask[0:h,w/2:w]
        M_right = cv2.moments(mask_right)
        if M_right['m00'] > 0:
          cx2 = int(M_right['m10']/M_right['m00'])
          #cy = int(M['m00']/M['m01'])
          cy2 = int(M_right['m01']/M_right['m00'])
          cv2.circle(mask_right, (cx2, cy2), 20, (0,0,255), -1)

          newImage = mask_right.copy()

      elif(path == 2):
        mask_left =  mask[0:h,0:w/2]
        M_left = cv2.moments(mask_left)
        if M_left['m00'] > 0:
          cx = int(M_left['m10']/M_left['m00'])
          #cy = int(M['m00']/M['m01'])
          cy = int(M_left['m01']/M_left['m00'])

          cv2.circle(mask_left, (cx, cy), 20, (0,0,255), -1)

          newImage = mask_left.copy()
      
      newImage = ROI.copy()
      # CONTROL starts
      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 80

      rospy.logdebug("z error %f", -float(err) / 80)
      self.cmd_vel_pub.publish(self.twist)
  

      


      #finding contours 
      cnts = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
      cnts = cnts[0] if len(cnts) == 2 else cnts[1]

      #drawing Contours
      radius =2
      color = (30,255,50)
      #cv2.drawContours(newImage, cnts[0], -1,color , radius)


      msg = CompressedImage()
      msg.header.stamp = rospy.Time.now()
      msg.format = "jpeg"
      msg.data = np.array(cv2.imencode('.jpg', newImage)[1]).tostring()

      mask_pub = CompressedImage()
      mask_pub.header.stamp = rospy.Time.now()
      mask_pub.format = "jpeg"
      mask_pub.data = np.array(cv2.imencode('.jpg', ROI)[1]).tostring()


      self.image_comp_pub.publish(msg)
      self.mask_comp_pub.publish(mask_pub)
      # CONTROL ends


rospy.init_node('follower', log_level=rospy.DEBUG)
follower = Follower()
rospy.spin()
# END ALL