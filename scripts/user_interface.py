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
		
		if cmd == 1:
		
			print ('	Please type the coordinates:')
			
			x = float(input('		x :'))
			
			y = float(input('		y :'))
			
			client.wait_for_server()
			
			print ('	Trying to reach the point [{}, {}]'.format(x, y))
			
			
			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = 'map'
			goal.target_pose.pose.orientation.w = 1.0
			goal.target_pose.pose.position.x = x
			goal.target_pose.pose.position.y = y
			      			
			client.send_goal(goal)
			client.wait_for_result()
  			#client.get_result()
			
		elif cmd == 2:
			
			print ('	Ok')
			
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
