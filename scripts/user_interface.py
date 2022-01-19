#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import *
from std_srvs.srv import *
from final_assignment.msg import CommandMessage
import selectors #Experiment
import sys #Experiment

client = actionlib.SimpleActionClient('move_base', MoveBaseAction) #Experiment

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
			
			#print ('\n	Going to [{}, {}]'.format(x, y))
			
			
			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = 'map'
			goal.target_pose.pose.orientation.w = 1.0
			goal.target_pose.pose.position.x = x
			goal.target_pose.pose.position.y = y
				
			client.send_goal(goal)	
			seconds = 40
			
			#print("Waiting for the goal to be accepted.")
			while client.get_state() != 1:
				rate.sleep()
			print("\n	Goal accepted. Press something to abort", end='')
			while seconds > 0 and client.get_state() == 1:
				sel = selectors.DefaultSelector()
				sel.register(sys.stdin, selectors.EVENT_READ)
				#print("Want to abort? (y/n): ", end='')
				sys.stdout.flush()
				pairs = sel.select(timeout=5)
				seconds = seconds - 5
				if pairs:
					sys.stdin.readline().strip()
					#print('you entered:', passcode)
					client.cancel_goal()
					break
				else:
					print('\n	{} seconds before aborting'.format(seconds))
					
			if client.get_state() == 3:
				print('\n	Goal reached')
			elif client.get_state() == 4:
				print('\n	Aborted by the server: Not reachable')
			elif seconds <= 0:
				client.cancel_goal()
				print('\n	Aborted by the client: it has taken too much')
			else:
				print('\n	Aborted by the user')
				sys.stdout.flush()
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
