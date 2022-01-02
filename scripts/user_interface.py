#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import *
from std_srvs.srv import *
from final_assignment.msg import ManualControl

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

pub = rospy.Publisher('/middleman/control', ManualControl)

reset_control = ManualControl()
reset_control.ctrl = 0
reset_control.helper = 0
 
def main():
	global pub, active_

	rospy.init_node('user_interface')

	print ('\n\nWelcome, please type:\n	" 1 " to autonomously reach a point,\n	" 2 " to drive the robot with the keyboard,\n	" 3 " to drive the robot assisted by the computer,\n	" 4 " to close the simulation.')
    
	rate = rospy.Rate(20)
	
	while not rospy.is_shutdown():
	
		cmd = float(input('\nCommand :'))
		
		client.wait_for_server()
		client.cancel_all_goals()
		pub.publish(reset_control)
		
		if cmd == 1:
		
			print ('	Please type the coordinates:')
			
			x = float(input('		x :'))
			
			y = float(input('		y :'))
			
			#client.wait_for_server()
			
			print ('\n	Going to [{}, {}]'.format(x, y))
			
			
			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = 'map'
			goal.target_pose.pose.orientation.w = 1.0
			goal.target_pose.pose.position.x = x
			goal.target_pose.pose.position.y = y
			      	
			client.send_goal(goal)
			
			print ('\n	Press any number to cancel')
			
			a = client.wait_for_result(rospy.Duration(30))
			
			print(a)
			
		elif cmd == 2:
			
			print ('\n	You have the control')
			
			control_command = ManualControl()
			control_command.ctrl = 1
			control_command.helper = 0
			pub.publish(control_command)
			
			print ('	Press any number to cancel')	
		
		elif cmd == 3:
		
			print ('\n	You have partially the control')
		
			control_command = ManualControl()
			control_command.ctrl = 1
			control_command.helper = 1
			pub.publish(control_command)	
			
			print ('	Press any number to cancel')		
			
		elif cmd == 4:
			
			client.cancel_all_goals()
			
			return
		else:
		
			print ('	Wrong character, please type again.')
		
		rate.sleep()


if __name__ == '__main__':
    main()
