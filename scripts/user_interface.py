#! /usr/bin/env python

import rospy
from std_srvs.srv import *




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
			
			#set the move_base goal and wait giving the user the chance to cancel the action
			
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
