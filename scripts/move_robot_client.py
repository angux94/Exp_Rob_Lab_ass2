#! /usr/bin/env python
# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf import transformations
import math
import actionlib
import actionlib.msg
import motion_plan.msg
import time

desired_coords = Point()

def cb_move(point_data):
	global desired_coords
	desired_coords.x = point_data.x
	desired_coords.y = point_data.y
	desired_coords.z = point_data.z
	go_to_coords(desired_coords)

def go_to_coords(des_coords):
	global act_c
	coords = motion_plan.msg.PlanningGoal()

	coords.target_pose.pose.position.x = des_coords.x
	coords.target_pose.pose.position.y = des_coords.y
	coords.target_pose.pose.position.z = des_coords.z

	act_c.send_goal(coords)
	rospy.loginfo('Goal: ' + str(coords.target_pose.pose.position.x) + ', ' + str(coords.target_pose.pose.position.y))

	# Waits for the server to finish performing the action.
	act_c.wait_for_result()

def main():
	#global pub, active_, act_s
	rospy.init_node('move_robot_client')
	sub_coords = rospy.Subscriber('/move_coords', Point, cb_move)
	act_c = actionlib.SimpleActionClient('/move_goal', motion_plan.msg.PlanningAction)
	
	rospy.loginfo('Waiting for action server to start')
	act_c.wait_for_server()
	rospy.loginfo('Server started')
	
	rate = rospy.Rate(20)
		
	while not rospy.is_shutdown():
		rate.sleep()


if __name__ == '__main__':
    main()
