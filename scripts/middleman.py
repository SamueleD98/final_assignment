#! /usr/bin/env python

import rospy
#from std_msgs.msg import String, Empty
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from final_assignment.msg import ManualControl

pub = rospy.Publisher("/cmd_vel", Twist)

teleop_status = False
helper_status = False

desired_speed = {
	'linear':  0,
	'angular':  0,
}


def clbk_laser(msg):
	    
	regions = {
		'right':  min(min(msg.ranges[0:287]), 10),
		'front': min(min(msg.ranges[288:431]), 10),
		'left':  min(min(msg.ranges[432:719]), 10),
  	}
    
	if helper_status:
		check_vel(regions)
		

def check_vel(regions):
	
	global desired_speed
	
	vel = Twist()	
	vel.linear.x = desired_speed['linear'] 
	vel.angular.z = desired_speed['angular'] 
	
	if regions['front'] < 1 and vel.linear.x > 0:
		vel.linear.x = 0.0
   	
	if (regions['left'] < 1 and vel.angular.z > 0) or (regions['right'] < 1 and vel.angular.z < 0):
		vel.angular.z = 0.0
   	
	pub.publish(vel)	
	

def control_switch(cmd):
	
	global teleop_status, helper_status
	
	teleop_status = cmd.ctrl
	helper_status = cmd.helper
	
	
def new_vel(vel):

	global teleop_status, helper_status, desired_speed

	if teleop_status:
		if helper_status:
			desired_speed['linear'] = vel.linear.x
			desired_speed['angular'] = vel.angular.z
		else:				
			pub.publish(vel)	
			

def main():

	rospy.init_node("middleman")
	
	rospy.Subscriber('/scan', LaserScan, clbk_laser)
	
	rospy.Subscriber('/middleman/control', ManualControl, control_switch) 
	
	rospy.Subscriber('/middleman/cmd_vel', Twist, new_vel) 	
	
	rospy.spin()


if __name__ == '__main__':
    main()
    
    
    
    
