#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import random
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose
import numpy as np
import matplotlib.pyplot as plt




# Global variables
cb_msg = None


def coord_select():
	"""Function to display a grid and select the destination point

	Returns
	---------
	xy
		X and Y coordinates of the point selected
	"""

	# 8x8 grid for selecting the destination
	plt.title('Select play destination')
	plt.figure(1)
	for i in range(-8,8):
		plt.plot([i, i], [-8,8], 'k')
		plt.plot([-8,8],[i, i], 'k')

	# Point input
	xy = plt.ginput(1)
	plt.close()
	plt.show()

	return xy

# Callback functions
def callback(data):
	global cb_msg
	cb_msg = data.data
	

def main():
	
	"""Main code for gesture recognition

	After receiving the "play" command, displays a 16x16 grid as the environment to 
        select a play destination. Publishes the selected coordinates
	
	Subscribers
	----------
	sub: subscriber (std_msgs.String) to /gesture_request
		reads when the play command arrives

	Publishers
	----------
	pub: publisher (geometry_msgs.Point) to /ball_coords
		publishes the ball selected desired coordinates
	"""
	rospy.init_node('play_coords')

	# Publishers and Subscribers
	pub = rospy.Publisher('/ball_coords', Point, queue_size=10)
	sub = rospy.Subscriber('/gesture_request', String, callback)

	# Initialiizations
	global cb_msg

	play_coord = Point(x = 0, y = 0, z = 0)

	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		
		# Check if we have a command
		if(cb_msg == "play"):
			# Clean variable
			cb_msg = None

			# Get destination coordinates
			xy = coord_select()

			#storage of the x and y coords of the point
			x = xy[0][0]
			y = xy[0][1]
			z = 0.5

			# Publish coordinates
			play_coord = Point(x = x, y = y, z = z)
			pub.publish(play_coord)

		if(cb_msg == "stop"):
			# Clean variable
			cb_msg = None
			x = 0
			y = 0
			z = -2
			# Publish coordinates to hide ball
			play_coord = Point(x = x, y = y, z = z)
			pub.publish(play_coord)

			
		rate.sleep()
	rospy.spin()

if __name__ == '__main__':
	main()
