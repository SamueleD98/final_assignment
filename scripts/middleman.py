#! /usr/bin/env python

import rospy
import actionlib
#from std_msgs.msg import String, Empty
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from final_assignment.msg import CommandMessage
from final_assignment.msg import FeedbackMessage
from move_base_msgs.msg import *

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

pub = rospy.Publisher("/cmd_vel", Twist)

fdbk = rospy.Publisher('/middleman/feedback', FeedbackMessage) #here?

keyboard_status = False
helper_status = False

desired_speed = {
	'linear':  0,
	'angular':  0,
}



goal_exist = 0




def clbk_laser(msg):
	if helper_status:
		regions = {
			'right':  min(min(msg.ranges[0:287]), 10),
			'front': min(min(msg.ranges[288:431]), 10),
			'left':  min(min(msg.ranges[432:719]), 10),
	  	}	
		update_vel(regions)		

def update_vel(regions):
	vel = Twist()	
	
	if regions['front'] < 1 and vel.linear.x > 0:
		vel.linear.x = 0.0
	else:
		vel.linear.x = desired_speed['linear'] 
   	
	if (regions['left'] < 1 and vel.angular.z > 0) or (regions['right'] < 1 and vel.angular.z < 0):
		vel.angular.z = 0.0
	else:
		vel.angular.z = desired_speed['angular'] 
		
	pub.publish(vel)		
	

def commandCallBack(cmd):
	
	global keyboard_status, helper_status, desired_speed, client, goal_exist #!!!!!!!!!!!!!!!
	
	#Each time a new command is given, 
	#the node cancel the last goal and set the speed to 0
	
	#client.cancel_all_goals()

	reset()
	
	goal_exist = 0
	
	if cmd.enable_taxi:
		go_to(cmd.des_x, cmd.des_y)
	else:
		keyboard_status = cmd.enable_userCtrl
		helper_status = cmd.enable_helper	
		
def reset():
	global desire_speed, client, pub
	
	client.wait_for_server()	#??????????'
	client.cancel_goal()
	
	desired_speed['linear'] = 0
	desired_speed['angular'] = 0
	
	vel = Twist()	
	vel.linear.x = 0
	vel.angular.z = 0
	pub.publish(vel)	
			
def go_to(x, y):	
	
	#rate = rospy.Rate(20) #here?
	
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.pose.orientation.w = 1.0
	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	
	goal_exist = 1	
	client.send_goal(goal)	
		
	
	#if not client.wait_for_result(rospy.Duration(30)):
		#client.cancel_goal()
		#vel = Twist()	
		#vel.linear.x = 0.99999
		#vel.angular.z = 0.99999
		#pub.publish(vel)	
			
	seconds = 1255
	
	status = FeedbackMessage()
	
	while client.get_state() != 1:
		#print("\n	Waiting for the goal to be accepted.")
		
		status.feedback = 1
		fdbk.publish(status)
		rate.sleep()
	#print("\n	Goal accepted. Press something to abort", end='')
	
	goal_exist = 1
	
	
	

			
def new_vel(vel):

	global desired_speed

	if keyboard_status:
		if helper_status:
			desired_speed['linear'] = vel.linear.x
			desired_speed['angular'] = vel.angular.z
		else:				
			pub.publish(vel)	
	
	
def main():
	
	global goal_exist, seconds

	rospy.init_node("middleman")
	
	rate = rospy.Rate(1) #here?
	
	rospy.Subscriber('/scan', LaserScan, clbk_laser)
	
	rospy.Subscriber('/middleman/control', CommandMessage, commandCallBack) 
	
	rospy.Subscriber('/middleman/cmd_vel', Twist, new_vel) 	
	
	while not rospy.is_shutdown():
	
	
		print(' Goal: {}'.format(goal_exist))
	
		if goal_exist:
			#seconds = 1255
			
			status = FeedbackMessage()			
			
			if seconds > 0 and client.get_state() == 1:
				status.time_left = seconds
				status.feedback = 2
				fdbk.publish(status)
				seconds = seconds - 1		
			else:
				if client.get_state() == 3:
					status.feedback = 3
					fdbk.publish(status)
					print('\n	Goal reached')
				elif client.get_state() == 4:
					status.feedback = 4
					fdbk.publish(status)
					print('\n	Aborted by the server: Not reachable')
				elif seconds <= 0:
					status.feedback = 5
					fdbk.publish(status)
					client.cancel_goal()
					print('\n	Aborted by the client: it has taken too much')
				else:
					status.feedback = 6
					fdbk.publish(status)
					print('\n	Uknown') #?????????????
					
				goal_exist = 0
		rate.sleep()
			
	
	rospy.spin()


if __name__ == '__main__':
    main()
    
    
    
    
