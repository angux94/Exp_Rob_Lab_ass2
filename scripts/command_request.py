#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import random
from std_msgs.msg import String
from geometry_msgs.msg import Point
import numpy as np
import matplotlib.pyplot as plt


def main():
	"""Main code for command recognition

	Waits for the user to input a command for the program

	subscribe: None
	publish: /command (std_msgs.String)

	Valid comands: Play, PLAY, play
	"""
	rospy.init_node('command_request')

	# Publishers and subscribers
	pub = rospy.Publisher('/command', String, queue_size=10)
	
	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():

		# Get command from user
		txt = raw_input("Write play or stop to put the ball in the environment: \n")
		if(txt == "Play" or txt == "play" or txt == "PLAY"):
			txt = "play"
			pub.publish(txt)
		
		elif(txt == "Stop" or txt == "stop" or txt == "STOP"):
			print("Ball disappears!")
			txt = "stop"
			pub.publish(txt)
		# Code for invalid comand and retry
		else:
			print("Your command '" + txt + "' is not valid.")
			print("Please write a valid command")
			print("")
			continue
		rate.sleep()

	rospy.spin()

if __name__ == '__main__':
	main()
