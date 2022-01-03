#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import *
from std_srvs.srv import *
from final_assignment.msg import CommandMessage

command = rospy.Publisher('/middleman/control', CommandMessage)
 
def main():
		
	rospy.init_node('user_interface')

	print ('\n\nWelcome, please type:\n	" 1 " to autonomously reach a point,\n	" 2 " to drive the robot with the keyboard,\n	" 3 " to drive the robot assisted by the computer,\n	" 4 " to close the simulation.')
    
	rate = rospy.Rate(20)
	
	while not rospy.is_shutdown():
	
		try:
			cmd = float(input('\nCommand :'))
		except:
			print ('	Wrong character, please type again.')
			continue
			
		control_command = CommandMessage()
		control_command.enable_taxi = False
		control_command.des_x = 0
		control_command.des_y = 0
		control_command.enable_userCtrl = False
		control_command.enable_helper = False		
		
		# Reset the configuration, canceling every past command
		command.publish(control_command)
		
		if cmd == 1:
		
			print ('	Please type the coordinates:')
			
			x = float(input('		x :'))
			
			y = float(input('		y :'))
			
			#client.wait_for_server()
			
			print ('\n	Going to [{}, {}]'.format(x, y))
			
			control_command.enable_taxi = True
			control_command.des_x = x
			control_command.des_y = y
			
			command.publish(control_command)
			
			print ('\n	Cancel by giving another command or by pressing " 0 "')	
			
		elif cmd == 2:
			
			print ('\n	You have the control')
			
			control_command.enable_userCtrl = True
			
			command.publish(control_command)
			
			print ('\n	Cancel by giving another command or by pressing " 0 "')	
		
		elif cmd == 3:
		
			print ('\n	You have partially the control')
		
			control_command.enable_userCtrl = True
			control_command.enable_helper = True
			
			command.publish(control_command)	
			
			print ('\n	Cancel by giving another command or by pressing " 0 "')	
			
		elif cmd == 4:
			
			return
			
		elif cmd == 0:
		
			print ('\n	Canceled')
			
		else:
		
			print ('	Please type again.')
		
		rate.sleep()


if __name__ == '__main__':
    main()
