#!/usr/bin/env python
#EPRobot 
import rospy, cv2, cv_bridge, numpy
import serial
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage



 
class Follower:
  def __init__(self):
    #=======================
    self.flag_g = 0
    self.flag_y = 0
    self.flag_n = 0
    self.flag_record_y = 0

    self.bridge = cv_bridge.CvBridge()

    #self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
    self.image_sub = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, self.image_callback, queue_size=10)
    #self.image_pub = rospy.Publisher('/image_hsv', Image, queue_size=10)
    #self.image_compressed_pub = rospy.Publisher('/image_hsv/compressed', CompressedImage, queue_size=10)
    #self.image_bin_pub = rospy.Publisher('/image_hsv/bin', Image, queue_size=10)
    #self.image_bin_compressed_pub = rospy.Publisher('/image_hsv/bin/compressed', CompressedImage, queue_size=10)
    #self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)
    #self.twist = Twist()


    self.forward_velocity = rospy.get_param('~forward_velocity', 0.2)
    self.scale_diversion = rospy.get_param('~scale_diversion', 0.4)
    
    # Astra Pro image offset = 60
    self.img_offset = rospy.get_param('~img_offset', 60)

 
  def image_callback(self, msg):
    #image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')


    image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
    image = cv2.GaussianBlur(image,(11,11),1)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_blue = numpy.array([20,100,46])
    upper_blue = numpy.array([34,255,255])
    
    lower_green = numpy.array([33,100,50])
    upper_green = numpy.array([73,255,255])
    
    mask_y= cv2.inRange(hsv, lower_blue, upper_blue) #
    mask_g  = cv2.inRange(hsv, lower_green, upper_green) #
    
    '''
    # BEGIN CROP
    h, w, d = image.shape
    search_top = 2*h/3
    search_bot = search_top + 20
    mask_y[0:search_top, 0:w] = 0
    mask_y[search_bot:h, 0:w] = 0
    # END CROP
    '''
    # BEGIN FINDER
    M_y = cv2.moments(mask_y)
    M_g = cv2.moments(mask_g)
    if M_y['m00'] > 50000000:
      if self.flag_y == 0:
        self.flag_n = 0
        self.flag_y = 1
        self.flag_record_y=1
        rospy.set_param("color_flag_sjq", 9)

        print 'yellow'
      self.color = 2
    elif M_g['m00'] > 50000000:
      if self.flag_g == 0:
        self.flag_n = 0
        self.flag_g = 1
        if self.flag_record_y ==0:
          rospy.set_param("color_flag_sjq", 1)
        print 'green'
      self.color = 1
    else:
      if self.flag_n == 0:
        self.flag_n = 1
        self.flag_g = 0
        self.flag_y = 0
        print 'No colour'
    
                 
    #mask = self.bridge.cv2_to_imgmsg(mask)
    #self.image_bin_pub.publish(mask)



 
rospy.init_node('follower')

follower = Follower()
rospy.spin()

