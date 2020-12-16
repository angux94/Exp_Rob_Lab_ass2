#!/usr/bin/env python

# Python libs
import sys
import time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String, Bool

VERBOSE = False
cb_msg = None

def wait_time(seconds):
	start_time = time.time()
	my_time = 0
	while (my_time < seconds):
		my_time = time.time()-start_time

class image_feature:

    def __init__(self):
	global cb_msg
        '''Initialize ros publisher, ros subscriber'''
        rospy.init_node('camera_ball', anonymous=True)
     # topic where we publish
        self.image_pub = rospy.Publisher("/robot/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("/robot/cmd_vel",
                                       Twist, queue_size=1)
	self.cam_ctrl_pub = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=1)

	self.arrived_pub = rospy.Publisher("/arrived_play",
                                       Bool, queue_size=1)

        # subscribed Topic
	self.sub = rospy.Subscriber('/gesture_request', String, self.cb_request)

        self.subscriber = rospy.Subscriber("/robot/camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)

	self.arrive = False

    def cb_request(self, data):
	global cb_msg
	cb_msg = data.data


    def callback(self, ros_data):
	global cb_msg
	
	'''Callback function of subscribed topic. 
	Here images get converted and features detected'''
	if VERBOSE:
	        print ('received image of type: "%s"' % ros_data.format)

	#### direct conversion to CV2 ####
	np_arr = np.fromstring(ros_data.data, np.uint8)
	image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

	greenLower = (50, 50, 20)
	greenUpper = (70, 255, 255)

	blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	#cv2.imshow('mask', mask)
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	                        cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None
	if cb_msg == 'play':
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
			cv2.circle(image_np, (int(x), int(y)), int(radius),
			           (0, 255, 255), 2)
			cv2.circle(image_np, center, 5, (0, 0, 255), -1)
			vel = Twist()
			vel.angular.z = -0.002*(center[0]-400)
			vel.linear.x = -0.01*(radius-100)
			self.vel_pub.publish(vel)
			#print distance when close enough
			if abs(radius-100) < 10:
				rospy.loginfo('dist: ' + str(radius-100))
			#check if we arrived to the point
			if(self.arrive == False):
				if(abs(radius-100) < 1):
					self.arrived_pub.publish(True)
					self.arrive = True

					cam_angle = Float64()
					angle = Float64()
					angle = 0.0

					while angle < 0.78:
						angle = angle + 0.1
						#rospy.loginfo('angle: ' + str(angle))
						cam_angle.data = angle
						self.cam_ctrl_pub.publish(cam_angle)
						cv2.imshow('window', image_np)
						cv2.waitKey(2)
						time.sleep(1)
				
					wait_time(5)


					while angle > -0.78:
						angle = angle - 0.1
						#rospy.loginfo('angle: ' + str(angle))
						cam_angle.data = angle
						self.cam_ctrl_pub.publish(cam_angle)
						cv2.imshow('window', image_np)
						cv2.waitKey(2)
						time.sleep(1)

					wait_time(5)

					while angle < -0.1:
						angle = angle + 0.1
						#rospy.loginfo('angle: ' + str(angle))
						cam_angle.data = angle
						self.cam_ctrl_pub.publish(cam_angle)
						cv2.imshow('window', image_np)
						cv2.waitKey(2)
						time.sleep(1)

					self.arrived_pub.publish(False)
					rospy.loginfo('Checked around!')
				

				

		    else:
			vel = Twist()
			vel.linear.x = 0.5
			self.vel_pub.publish(vel)

		else:
		    vel = Twist()
		    vel.angular.z = 0.5
		    self.vel_pub.publish(vel)
		    self.arrive = False

		cv2.imshow('window', image_np)
		cv2.waitKey(2)

	elif cb_msg == 'stop':
		start_time = time.time()
		my_time = 0
		while (my_time < 10):
			my_time = time.time()-start_time
			vel = Twist()
			vel.angular.z = 0.5
			self.vel_pub.publish(vel)
			cv2.imshow('window', image_np)
			cv2.waitKey(2)
		cb_msg = None

	else:
		cv2.imshow('window', image_np)
		cv2.waitKey(2)
	# self.subscriber.unregister()


def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
