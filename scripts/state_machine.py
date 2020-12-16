#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String, Bool
import random
from geometry_msgs.msg import Point
from tf import transformations
import math
import actionlib
import actionlib.msg
import motion_plan.msg
import time

# define state NORMAL
class Normal(smach.State):
    """ Class for the NORMAL state

    Robot walks randomly for a random amount of times.
    If "play" command is received, goes to PLAY state.
    If no command is received, goes to sleep.

    Attributes
    ----------
    normal_counter: int
    pub: publisher (geometry_msgs.Point) to /move_coords
    sub_command: subscriber (std_msgs.String) to /command
    sub_flag: subscriber (std_msgs.Bool) to /arrived
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['play','sleep'])
               


        #Publishers and subscribers
        self.pub = rospy.Publisher('/move_coords', Point, queue_size=10)
        self.sub_command = rospy.Subscriber('/command', String, cb_command)
        self.sub_flag = rospy.Subscriber('/arrived', Bool, cb_flag)


	self.act_c = actionlib.SimpleActionClient('/move_goal', motion_plan.msg.PlanningAction)
	
	rospy.loginfo('Waiting for action server to start')
	self.act_c.wait_for_server()
	rospy.loginfo('Server started')

        # Initializations
        self.normal_counter = 1
	self.coords = motion_plan.msg.PlanningGoal() 

    def execute(self, userdata):

        global sm_command, sm_flag

        # Restart the counter every time
        self.normal_counter = 1
        time.sleep(1)
        rospy.loginfo('Executing state NORMAL')

        # Check if there is previously a command in the buffer
        if sm_command == "play":
            print(sm_command)
            sm_command = None
            return 'play'
        
        # If not, proceed to randomly walk
        else:
	    # Amount of random walks before sleeping
            normal_times = rospy.get_param('~normal_times',random.randrange(1,5))

            x = random.randrange(-8,8)
            y = random.randrange(-8,8)
	    z = 0

            normal_coord = Point(x = x, y = y, z = z)

	    self.coords.target_pose.pose.position.x = normal_coord.x
	    self.coords.target_pose.pose.position.y = normal_coord.y
	    self.coords.target_pose.pose.position.z = normal_coord.z

            # Status control
            print("Robot acting normal")
            print("Times: " + str(normal_times))
            print("Counter: " + str(self.normal_counter))
            print('Coords: ' + str(x) + ', ' + str(y))
            print('--------------------------')

	    #Go to the generated coords
	    self.act_c.send_goal(self.coords)
	    rospy.loginfo('Goal: ' + str(self.coords.target_pose.pose.position.x) + ', ' + str(self.coords.target_pose.pose.position.y))

	    # Waits for the server to finish performing the action.
	    self.act_c.wait_for_result()

            while not rospy.is_shutdown():
		# Check if there was a play command in between random walks
                if sm_command == "play":
		    print(sm_command)
                    sm_command = None
                    return 'play'

                # If not, continue with the behavior
                if(self.normal_counter < normal_times):

                    x = random.randrange(-8,8)
                    y = random.randrange(-8,8)
		    z = 0

                    normal_coord = Point(x = x, y = y, z = z)
                    
		    self.coords.target_pose.pose.position.x = normal_coord.x
	    	    self.coords.target_pose.pose.position.y = normal_coord.y
	    	    self.coords.target_pose.pose.position.z = normal_coord.z

                    self.normal_counter = self.normal_counter + 1

                    # Status control
                    print("Times: " + str(normal_times))
                    print("Counter: " + str(self.normal_counter))
                    print('Coords: ' + str(x) + ', ' + str(y))
                    print('--------------------------')

		    #Go to the generated coords
	    	    self.act_c.send_goal(self.coords)
	    	    rospy.loginfo('Goal: ' + str(self.coords.target_pose.pose.position.x) + ', ' + str(self.coords.target_pose.pose.position.y))

	    	    # Waits for the server to finish performing the action.
	    	    self.act_c.wait_for_result()

                else: return 'sleep'



# define state SLEEP
class Sleep(smach.State):
    """ Class for the SLEEP state

    The robot goes to the sleep coordinate, stays there for a while, then wakes up and goes to NORMAL state

    Parameters
    ----------
    sleep_x: (int) Sleep x coordinate (-8-8)
    sleep_y: (int) Sleep y coordinate (-8-8)
    time_sleep: (int) Sleeping time (1-10)

    Attributes
    ----------
    pub: publisher (geometry_msgs.Point) to /move_coords
    sub_flag: subscriber (std_msgs.Bool) to /arrived
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait'])      

        #Publishers and subscribers
        self.pub = rospy.Publisher('move_coords', Point, queue_size=10)
        self.sub_flag = rospy.Subscriber('arrived', Bool, cb_flag)

	self.act_c = actionlib.SimpleActionClient('/move_goal', motion_plan.msg.PlanningAction)

	rospy.loginfo('Waiting for action server to start')
	self.act_c.wait_for_server()
	rospy.loginfo('Server started')

	#Initializations
	self.coords = motion_plan.msg.PlanningGoal()  

    def execute(self, userdata):
        time.sleep(1)
        rospy.loginfo('Executing state SLEEP')

        global sm_flag

        # Coordinates of the sleep position
        sleep_x = rospy.get_param('~sleep_x', 7)
        sleep_y = rospy.get_param('~sleep_y', 1)
	sleep_z = 0
	
	sleep_coord = Point(x = sleep_x, y = sleep_y, z = sleep_z)

	self.coords.target_pose.pose.position.x = sleep_coord.x
	self.coords.target_pose.pose.position.y = sleep_coord.y
	self.coords.target_pose.pose.position.z = sleep_coord.z

        # Go to sleep position
	print("going to sleep")
	self.act_c.send_goal(self.coords)
	rospy.loginfo('Goal: ' + str(self.coords.target_pose.pose.position.x) + ', ' + str(self.coords.target_pose.pose.position.y))

	# Waits for the server to finish performing the action.
	self.act_c.wait_for_result()

	time_sleep = rospy.get_param('~time_sleep', 10)

        while not rospy.is_shutdown():

            # When arrived, sleep for a fixed time and then continue to NORMAL state
            print("Robot arrived to sleep")
            time.sleep(time_sleep)
            print("Robot woken")
            return 'wait'
        


# define state PLAY
class Play(smach.State):
    """ Class for the PLAY state

    Robot plays for a random amount of times.
    Each time goes towards the man and waits for a gesture, then goes towards the indicated coords and repeats.
    When it finishes playing goes to NORMAL state.

    Parameters
    ----------
    man_x: (int) Man x coordinate (0-1000)
    man_y: (int) Man y coordinate (0-1000)
    play_times: (int) Amount of times to play (1-5)


    Attributes
    ----------
    play_counter: int
    pub_command: publisher (std_msgs.String) to /gesture_request
    pub_coords: publisher (geometry_msgs.Point) to /move_coords
    sub_flag: subscriber (std_msgs.Bool) to /arrived
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait'])

        # Initialization
        self.play_counter = 1

        #Publishers and subscribers
        self.pub_command = rospy.Publisher('/gesture_request', String, queue_size=10)
        self.pub_coords = rospy.Publisher('/move_coords', Point, queue_size=10)
	
	self.sub_command = rospy.Subscriber('/command', String, cb_command)
        self.sub_flag = rospy.Subscriber('/arrived_play', Bool, cb_flag)

    def execute(self, userdata):

	global sm_flag
        time.sleep(1)
        rospy.loginfo('Executing state PLAY')
	self.pub_command.publish("play")
	sm_flag = False
        while not rospy.is_shutdown():       
                
		if sm_flag:
			sm_flag = False
                	self.pub_command.publish("play")
                        time.sleep(1)
		
		if(sm_command == "stop"):
			self.pub_command.publish("stop")
			print("Ball dissapeared!")
			time.sleep(10)
			return 'wait'

    

# Callback functions
sm_command = None
sm_flag = None

def cb_command(data):
    global sm_command
    sm_command = data.data


def cb_flag(data):
    global sm_flag
    sm_flag = data.data



# main
def main():
    ''' State machine initialization

    Creates the state machine, add states and link their outputs.

    '''
    global sm_command, sm_flag

    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['COORDS'])


    rate = rospy.Rate(10) # 10hz
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                                transitions={'play':'PLAY', 
                                            'sleep':'SLEEP'})

        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'wait':'NORMAL'})

        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'wait':'NORMAL'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()


    # Execute the state machine
    outcome = sm.execute()

    
    # Wait for ctrl-c to stop the application

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
