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

#Global variables
desired_position = Point()

def callback(point_data):
	global desired_position
	desired_position.x = point_data.x
	desired_position.y = point_data.y
	desired_position.z = point_data.z
	go_to_point(desired_position)

def go_to_point(des_position):
	global act_c
	goal = motion_plan.msg.PlanningGoal()

	goal.target_pose.pose.position.x = des_position.x
	goal.target_pose.pose.position.y = des_position.y
	goal.target_pose.pose.position.z = des_position.z

	act_c.send_goal(goal)
	rospy.loginfo('Goal: ' + str(goal.target_pose.pose.position.x) + ', ' + str(goal.target_pose.pose.position.y)+ ', ' + str(goal.target_pose.pose.position.z))

	# Waits for the server to finish performing the action.
	act_c.wait_for_result()
	

def main():
	global act_c
	rospy.init_node('move_ball_client')

	sub = rospy.Subscriber('/ball_coords', Point, callback)

	act_c = actionlib.SimpleActionClient('/reaching_goal', motion_plan.msg.PlanningAction)
	
	rospy.loginfo('Waiting for action server to start')
	act_c.wait_for_server()
	rospy.loginfo('Server started')
	
	rate = rospy.Rate(20)
		
	while not rospy.is_shutdown():
		rate.sleep()


if __name__ == '__main__':
    main()
