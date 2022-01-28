#! /usr/bin/env python

import rospy
import actionlib
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from final_assignment.msg import CommandMessage
from move_base_msgs.msg import *

pub = rospy.Publisher("/cmd_vel", Twist)

keyboard_status = False	#Enable the keyboard-given control commands
helper_status = False	#Enable the driving assistant

desired_speed = {'linear':  0, 'angular':  0}	#Desired velocity due to keyboard-given control commands


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
	
	global desired_speed, keyboard_status, helper_status
	
	#Each time a new command is given, 
	#the node set the speed to 0	
	desired_speed['linear'] = 0
	desired_speed['angular'] = 0
	
	vel = Twist()	
	vel.linear.x = 0
	vel.angular.z = 0
	pub.publish(vel)
	
	keyboard_status = cmd.enable_userCtrl
	helper_status = cmd.enable_helper				
			
def new_vel(vel):

	global desired_speed

	if keyboard_status:
		if helper_status:
			desired_speed['linear'] = vel.linear.x
			desired_speed['angular'] = vel.angular.z
		else:				
			pub.publish(vel)	
	
def main():

	rospy.init_node("middleman")
	
	rospy.Subscriber('/scan', LaserScan, clbk_laser)
	
	rospy.Subscriber('/middleman/control', CommandMessage, commandCallBack) 
	
	rospy.Subscriber('/middleman/cmd_vel', Twist, new_vel) 	
	
	rospy.spin()


if __name__ == '__main__':
    main()
    
    
    
    
