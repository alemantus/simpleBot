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

   #self.upperThreshold = rospy.set_param('image_proc/upperThreshold', 255)
  #self.lowerThreshold = rospy.set_param('/image_proc/lowerThreshold', 200)


  def image_callback(self, msg):

    #self.upperThreshold = rospy.get_param('image_proc/upperThreshold')
    #self.lowerThreshold = rospy.get_param('image_proc/lowerThreshold')

    self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

    #image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE) 
    #hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
    # change below lines to map the color you wanted robot to follow
    #lower_yellow = numpy.array([ 10,  10,  10])
    #upper_yellow = numpy.array([255, 255, 250])
    #mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    img_gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
    img_gray = cv2.GaussianBlur(img_gray, (7, 7), 0)
    (T, mask) = cv2.threshold(img_gray, 200, 255,cv2.THRESH_BINARY)
    
    h, w, d = self.image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(self.image, (cx, cy), 20, (0,0,255), -1)
      # CONTROL starts
      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 80

      rospy.logdebug("z error %f", -float(err) / 80)
      self.cmd_vel_pub.publish(self.twist)



      msg = CompressedImage()
      msg.header.stamp = rospy.Time.now()
      msg.format = "jpeg"
      msg.data = np.array(cv2.imencode('.jpg', self.image)[1]).tostring()

      mask_pub = CompressedImage()
      mask_pub.header.stamp = rospy.Time.now()
      mask_pub.format = "jpeg"
      mask_pub.data = np.array(cv2.imencode('.jpg', mask)[1]).tostring()


      self.image_comp_pub.publish(msg)
      self.mask_comp_pub.publish(mask_pub)
      # CONTROL ends


rospy.init_node('follower', log_level=rospy.DEBUG)
follower = Follower()
rospy.spin()
# END ALL