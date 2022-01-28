#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from final_assignment.msg import CommandMessage

pub = rospy.Publisher("/cmd_vel", Twist)	#Publisher of the velocity commands

keyboard_status = False	#Enable the keyboard-given control commands
helper_status = False	#Enable the driving assistant

desired_speed = {'linear':  0, 'angular':  0}	#Desired velocity due to keyboard-given control commands

def clbk_laser(msg):	#Each time a new laser scan is received
	if helper_status:	#If the assistant is enabled update the regions
		regions = {
			'right':  min(min(msg.ranges[0:287]), 10),
			'front': min(min(msg.ranges[288:431]), 10),
			'left':  min(min(msg.ranges[432:719]), 10),
	  	}	
		update_vel(regions)	#update the velocity	

def update_vel(regions):	# The velocity is updated depending on the close obstacles and on the desired speed
	vel = Twist()	
	
	
	if regions['front'] < 1 and desired_speed['linear'] > 0:	#If there are close obstacle in the front sector
		vel.linear.x = 0.0	# set the linear speed to zero
	else:	
		vel.linear.x = desired_speed['linear']	#the robot drive at the desired speed
   	
	if (regions['left'] < 1 and desired_speed['angular'] > 0) or (regions['right'] < 1 and desired_speed['angular']  < 0):	#If there are close obstacle in the lateral sector
		vel.angular.z = 0.0	# set the angular speed to zero
	else:
		vel.angular.z = desired_speed['angular']	#the robot drive at the desired speed
		
	pub.publish(vel)		
	

def commandCallBack(cmd):	#Each time a new command from the UI is received 
	
	global desired_speed, keyboard_status, helper_status
	vel = Twist()
	
	#set both the desired speed and the actual speed to 0	
	desired_speed['linear'] = 0
	desired_speed['angular'] = 0		
	vel.linear.x = 0
	vel.angular.z = 0
	pub.publish(vel)
	
	#Set the booleans as requested by the user
	keyboard_status = cmd.enable_userCtrl
	helper_status = cmd.enable_helper			
			
def new_vel(vel):	#Each time a new command from the keyboard is received

	global desired_speed

	if keyboard_status:	#Do something only if the keyboard it enabled
	
		if helper_status:	#If the assistant is enabled, just update the desired speed
			desired_speed['linear'] = vel.linear.x
			desired_speed['angular'] = vel.angular.z
		else:	
			pub.publish(vel)	#publish the new speed without any modifications	
	
def main():

	rospy.init_node("middleman")
	
	rospy.Subscriber('/scan', LaserScan, clbk_laser)
	
	rospy.Subscriber('/middleman/control', CommandMessage, commandCallBack) 
	
	rospy.Subscriber('/middleman/cmd_vel', Twist, new_vel) 	
	
	rospy.spin()


if __name__ == '__main__':
    main()
    
    
    
    
