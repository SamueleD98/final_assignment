#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import *
from std_srvs.srv import *

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
 
def main():
	global pub, active_

	rospy.init_node('user_interface')

	print ('\n\nWelcome, please type:\n	" 1 " to autonomously reach a point,\n	" 2 " to drive the robot with the keyboard,\n	" 3 " to drive the robot assisted by the computer,\n	" 4 " to close the simulation.')
    
	rate = rospy.Rate(20)
	
	while not rospy.is_shutdown():
	
		cmd = float(input('\nCommand :'))
		
		print(cmd)
		
		if cmd == 1:
		
			print ('	Please type the coordinates:')
			
			x = float(input('		x :'))
			
			y = float(input('		y :'))
			
			client.wait_for_server()
			
			print ('	Lets go to [{}, {}]'.format(x, y))
			
			
			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = 'map'
			goal.target_pose.pose.orientation.w = 1.0
			goal.target_pose.pose.position.x = x
			goal.target_pose.pose.position.y = y
			      	
			client.send_goal(goal)
			
			#while not finished
				#finished = client.wait_for_result(rospy.Duration(1*30))
			
			#while 1:
			#	print (a)
			#	while a == client.get_goal_status_text():
			#		b = 1
			#	a = client.get_goal_status_text()

			#print ('	Trying to reach the point [{}, {}].\nPress " 5 " to cancel the command.'.format(x, y))
			# Due modi:
				#1: aspetto in ui che completi in un tempo fissato, user non può agire
				#2: un altro nodo aspetta che il coso agisca
			#print ('	Reached')
			#bool finished_before_timeout = client.wait_for_result(rospy.Duration(1*30))
  			#client.get_result()
			
		elif cmd == 2:
			
			print ('	Ok')
			system('rosrun teleop_twist_keyboard teleop_twist_keyboard.py')
			
			#find a way to reproduce the teleop thing
		
		elif cmd == 3:
			
			print ('	Ok')
			
			#find a way to reproduce the teleop thing, you want the cpu to bypass user commands if they are suicidal
			
		elif cmd == 4:
			
			return
		else:
		
			print ('	Wrong character, please type again.')
		
		rate.sleep()


if __name__ == '__main__':
    main()
