#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import *
from final_assignment.msg import CommandMessage
import selectors 
import sys 

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)	#Move_Base action client

def wait():	#Wait for the goal to be accomplished or canceled
	
	rate = rospy.Rate(150)
	
	if rospy.has_param('countdown'):	#If the parameter exist, use it
		countdown = rospy.get_param('countdown')
	else:	#Default value
		countdown = 150
	
	while client.get_state() != 1:	#Wait until the goal is accepted
		rate.sleep()
		
	print('\n	Goal accepted by the server. Press enter to abort.\n\n	{} seconds before aborting'.format(countdown))
	
	while client.get_state() == 1:	#While the goal is active
		sel = selectors.DefaultSelector()	#Define selector
		sel.register(sys.stdin, selectors.EVENT_READ)	#Register the Read Event on the stdin input
		try_to_read = sel.select(timeout=1)	#Try to get an input for a second
		if try_to_read:	#If an input is given (and so the enter button has been pressed)
			sys.stdin.readline().strip() #Remove the line from the buffer
			client.cancel_goal()
			print('\n	Aborted by the user')
			return
		else:
			countdown = countdown - 1	#Decrease the timer variable
			if countdown <= 0:	#Time is over
				client.cancel_goal()
				print('\n	Aborted by the client: it has taken too much')
				return
			else:
				if (countdown%5 == 0) or (countdown < 5):	#Print the countdown every 5 values and the last 5 ones
					print('	{} seconds before aborting'.format(countdown))
		rate.sleep()
		
	if client.get_state() == 3:
		print('\n	Goal reached')
	elif client.get_state() == 4:
		print('\n	Aborted by the server: Not reachable')
	else:
		print('\n	Aborted by msg published on the /move_base/cancel topic') 


def main():	
	rospy.init_node('user_interface')
	
	rate = rospy.Rate(20)
	
	command = rospy.Publisher('/middleman/control', CommandMessage) #Used to send messages to the middleman node

	print('\n\nWelcome, please type:\n	" 1 " to autonomously reach a point,\n	" 2 " to drive the robot with the keyboard,')
	print('	" 3 " to drive the robot assisted by the computer,\n	" 4 " to close the simulation.')
    
	while not rospy.is_shutdown():
	
		try:
			cmd = int(input('\n Command :'))
		except:
			print ('\n	Wrong input, not an integer.')
			continue
		
		#Define command message with everything set to false
		control_command = CommandMessage()
		control_command.enable_userCtrl = False
		control_command.enable_helper = False		
		
		# Reset the configuration, canceling every past command
		command.publish(control_command)
		client.wait_for_server()		
		client.cancel_all_goals()
		
		if cmd == 1:	#Drive to a given point
		
			print ('	Please type the coordinates:')
			try:
				x = float(input('		x :'))
				y = float(input('		y :'))
			except:
				print ('\n	Wrong input, not a float.')
				continue
			
			print ('\n	Going to [{}, {}]'.format(x, y))
			
			#Define and set the goal message
			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = 'map'
			goal.target_pose.pose.orientation.w = 1.0
			goal.target_pose.pose.position.x = x
			goal.target_pose.pose.position.y = y
				
			client.send_goal(goal)	
			
			#Wait for the goal to be achieved or canceled
			wait()			
			
		elif cmd == 2:	#Drive freely
			
			print ('\n	You have the control')
			
			#Set and send the command message
			control_command.enable_userCtrl = True			
			command.publish(control_command)
			
			print ('	Cancel by giving another command or by pressing 0')	
		
		elif cmd == 3:	#Drive assisted by the CPU
		
			print ('\n	You have partially the control')
			
			#Set and send the command message
			control_command.enable_userCtrl = True
			control_command.enable_helper = True			
			command.publish(control_command)	
			
			print ('	Cancel by giving another command or by pressing 0')	
			
		elif cmd == 4:	#Exit the execution 
			
			return
			
		elif cmd == 0:	#Nothing, past command is canceled and the robot stop moving
		
			print ('\n	Canceled')
			
		else:
		
			print ('\n	Not a command, please type again.')
		
		rate.sleep()
		
		


if __name__ == '__main__':
    main()
