#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import *
from std_srvs.srv import *
from final_assignment.msg import CommandMessage
import selectors #Experiment
import sys #Experiment

command = rospy.Publisher('/middleman/control', CommandMessage)

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)



 
def main():
		
	rospy.init_node('user_interface')

	print ('\n\nWelcome, please type:\n	" 1 " to autonomously reach a point,\n	" 2 " to drive the robot with the keyboard,\n	" 3 " to drive the robot assisted by the computer,\n	" 4 " to close the simulation.')
    
	rate = rospy.Rate(20)
	
	while not rospy.is_shutdown():
	
		try:
			cmd = int(input('\n Command :'))
		except:
			print ('\n	Wrong input, not an integer.')
			continue
		
		control_command = CommandMessage()
		control_command.enable_userCtrl = False
		control_command.enable_helper = False		
		
		# Reset the configuration, canceling every past command
		command.publish(control_command)
		
		client.wait_for_server()	
		client.cancel_all_goals()
		
		if cmd == 1:
		
			print ('	Please type the coordinates:')
			try:
				x = float(input('		x :'))
				y = float(input('		y :'))
			except:
				print ('\n	Wrong input, not a float.')
				continue
			
			#client.wait_for_server()
			
			print ('\n	Going to [{}, {}]'.format(x, y))
			
			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = 'map'
			goal.target_pose.pose.orientation.w = 1.0
			goal.target_pose.pose.position.x = x
			goal.target_pose.pose.position.y = y
				
			client.send_goal(goal)	
			
			wait()
			
			
		elif cmd == 2:
			
			print ('\n	You have the control')
			
			control_command.enable_userCtrl = True
			
			command.publish(control_command)
			
			print ('	Cancel by giving another command or by pressing 0')	
		
		elif cmd == 3:
		
			print ('\n	You have partially the control')
		
			control_command.enable_userCtrl = True
			control_command.enable_helper = True
			
			command.publish(control_command)	
			
			print ('	Cancel by giving another command or by pressing 0')	
			
		elif cmd == 4:
			
			return
			
		elif cmd == 0:
		
			print ('\n	Canceled')
			
		else:
		
			print ('\n	Not a command, please type again.')
		
		rate.sleep()
def wait():

	rate = rospy.Rate(20) #here?
	
	if rospy.has_param('countdown'):
		countdown = rospy.get_param('countdown')
	else:
		countdown = 150
	
	while client.get_state() != 1:
		rate.sleep()
	print("\n	Goal accepted by the server. Press enter to abort")
	print('\n	{} seconds before aborting'.format(countdown))
	while client.get_state() == 1:
		sel = selectors.DefaultSelector()	#Define selector
		sel.register(sys.stdin, selectors.EVENT_READ)	#Register the Read Event on the stdin input
		try_to_read = sel.select(timeout=1)	#Try to get an input for a second
		if try_to_read:	#If an input is given (and so the enter button has been pressed)
			sys.stdin.readline().strip() #Remove the line from the buffer
			client.cancel_goal()
			print('\n	Aborted by the user')
			return
		else:
			countdown = countdown - 1
			if countdown <= 0:
				print('\n	Aborting..')
				client.cancel_goal()
				print('\n	Aborted by the client: it has taken too much')
				return
			else:
				if countdown%5 == 0:
					print('	{} seconds before aborting'.format(countdown))
		rate.sleep()
		
	if client.get_state() == 3:
		print('\n	Goal reached')
	elif client.get_state() == 4:
		print('\n	Aborted by the server: Not reachable')
	else:
		print('\n	Aborted by msg published on the /move_base/cancel topic') 

if __name__ == '__main__':
    main()
