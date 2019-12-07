#!/usr/bin/env python


import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import time

class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()
                cv2.namedWindow("window", 1)

                self.image_sub = rospy.Subscriber('camera/image_raw',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=1)
		
                self.twist = Twist()
                
                self.stop = Twist()
                self.stop.linear.x = 0
                self.stop.linear.y = 0
                self.stop.linear.z = 0
                self.stop.angular.x = 0
                self.stop.angular.y = 0
                self.stop.angular.z = 0
                
                self.stop_sign = False

        def image_callback(self, msg):
        
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                lower_yellow = np.array([ 10, 10, 10])
                upper_yellow = np.array([255, 255, 250])
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
                
                red_lower = np.array([0,25,110], dtype="uint8")
                red_upper = np.array([50,100,255], dtype="uint8")
                mask2 = cv2.inRange(hsv, red_lower, red_upper)
                
                h, w, d = image.shape
                
                stop_signs = cv2.HoughCircles(mask2[int(h/3):int(2*h/3),int(w/2):w],cv2.HOUGH_GRADIENT,1,20,
                                    param1=50,param2=20,minRadius=0,maxRadius=100)
                
                if stop_signs is not None:
                    self.stop_sign = True      
                    
                search_top = 3*h/4
                search_bot = 3*h/4 + 20
                mask[0:search_top, 0:w] = 0
                mask[search_bot:h, 0:w] = 0

                mass_center = cv2.moments(mask)
                if mass_center['m00'] > 0:
                    cx = mass_center['m10']/mass_center['m00']
                    err = cx - w + 10
                    self.twist.linear.x = 0.5
                    self.twist.linear.y = 0
                    self.twist.linear.z = 0
                    self.twist.angular.x = 0
                    self.twist.angular.y = 0
                    self.twist.angular.z = -float(err) / 100                
                        
                if (np.sum(mask2)==0 and self.stop_sign):
                    time.sleep(0.5)
                    self.cmd_vel_pub.publish(self.stop)
                else:
                    self.cmd_vel_pub.publish(self.twist)
                cv2.imshow("window", image)
                cv2.waitKey(3)
    

rospy.init_node('follower')
follower = Follower()
rospy.spin()
