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


if rospy.has_param('countdown'):
	countdown = rospy.get_param('countdown')
else:
	countdown = 5
 
def main():
		
	rospy.init_node('user_interface')

	print ('\n\nWelcome, please type:\n	" 1 " to autonomously reach a point,\n	" 2 " to drive the robot with the keyboard,\n	" 3 " to drive the robot assisted by the computer,\n	" 4 " to close the simulation.')
    
	rate = rospy.Rate(20)
	
	while not rospy.is_shutdown():
	
		cmd = float(input('\nCommand :'))
		
		control_command = CommandMessage()
		control_command.enable_taxi = False
		control_command.des_x = 0
		control_command.des_y = 0
		control_command.enable_userCtrl = False
		control_command.enable_helper = False		
		
		# Reset the configuration, canceling every past command
		command.publish(control_command)
		
		client.wait_for_server()	
		client.cancel_all_goals()
		
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
			
			wait()
			
			
		elif cmd == 2:
			
			print ('\n	You have the control')
			
			control_command.enable_userCtrl = True
			
			command.publish(control_command)
			
			print ('	Cancel giving another command or by pressing any other number')	
		
		elif cmd == 3:
		
			print ('\n	You have partially the control')
		
			control_command.enable_userCtrl = True
			control_command.enable_helper = True
			
			command.publish(control_command)	
			
			print ('	Press any number to cancel')		
			
		elif cmd == 4:
			
			return
			
		elif cmd == 0:
		
			print ('	Canceled')
			
		else:
		
			print ('	Wrong character, please type again.')
		
		rate.sleep()
def wait():

	rate = rospy.Rate(1) #here?
	
	#countdown = 150
	global countdown
	
	while client.get_state() != 1:
		rate.sleep()
	print("\n	Goal accepted. Press enter to abort\n\n", end='')
	while client.get_state() == 1:
		sel = selectors.DefaultSelector()
		sel.register(sys.stdin, selectors.EVENT_READ)
		#print("Want to abort? (y/n): ", end='')
		sys.stdout.flush()
		pairs = sel.select(timeout=1)
		
		if pairs:
			sys.stdin.readline().strip()
			#print('you entered:', passcode)
			
			client.cancel_goal()
			
			print('\n	Aborted by the user')
			return
		else:
			countdown = countdown - 1
			if countdown > 0:
				print('	{} seconds before aborting'.format(countdown))
	
			else:
				print('\n	Aborting..')
				client.cancel_goal()

		#print('\n	Goal reached')

		#print('\n	Aborted by the server: Not reachable')

		#print('\n	Aborted by the client: it has taken too much')
	
		#print('\n	Aborted by the user')
	if client.get_state() == 3:
		print('\n	Goal reached')
	elif client.get_state() == 4:
		print('\n	Aborted by the server: Not reachable')
	elif countdown <= 0:
		print('\n	Aborted by the client: it has taken too much')
	else:
		print('\n	Aborted') #Happened when canceling by publishing on cancel topic
	sys.stdout.flush()


if __name__ == '__main__':
    main()
