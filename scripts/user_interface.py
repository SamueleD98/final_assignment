#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import *
from std_srvs.srv import *
from final_assignment.msg import CommandMessage
from final_assignment.msg import FeedbackMessage
import selectors #Experiment
import sys #Experiment

client = actionlib.SimpleActionClient('move_base', MoveBaseAction) #Experiment

command = rospy.Publisher('/middleman/control', CommandMessage) #here?

feedback = 0
time_left = 99
 
def main():
		
	rospy.init_node('user_interface')
	
	rospy.Subscriber('/middleman/feedback', FeedbackMessage, feedbackCallBack) 

	print ('\n\nWelcome, please type:\n	" 1 " to autonomously reach a point,\n	" 2 " to drive the robot with the keyboard,\n	" 3 " to drive the robot assisted by the computer,\n	" 4 " to close the simulation.')
    
	rate = rospy.Rate(20)
	
	while not rospy.is_shutdown():
	
		try:
			cmd = float(input('\n Command :'))
		except:
			print ('\n	Wrong character, please type again.')
			continue
		
			
		control_command = CommandMessage()
		
		control_command.enable_taxi = False
		control_command.des_x = 0
		control_command.des_y = 0
		control_command.enable_userCtrl = False
		control_command.enable_helper = False		
		
		command.publish(control_command) # Reset the configuration, canceling every past command
		
		if cmd == 1:
		
			print ('	Please type the coordinates:')
			
			x = float(input('		x :'))
			
			y = float(input('		y :'))
			
			control_command.enable_taxi = True
			control_command.des_x = x
			control_command.des_y = y
			
			command.publish(control_command)
			
			wait()
			
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
		
	#spin ??

def feedbackCallBack(fdbk):
	global feedback, time_left
	
	feedback = fdbk.feedback
	
	time_left = fdbk.time_left

		
def wait():

	rate = rospy.Rate(20) #here?
	
	global feedback, time_left
	
	countdown = time_left
	
	while feedback != 2:
		rate.sleep()
	print("\n	Goal accepted. Press something to abort\n\n", end='')
	while feedback == 2:
		sel = selectors.DefaultSelector()
		sel.register(sys.stdin, selectors.EVENT_READ)
		#print("Want to abort? (y/n): ", end='')
		sys.stdout.flush()
		pairs = sel.select(timeout=1)
		
		if pairs:
			sys.stdin.readline().strip()
			#print('you entered:', passcode)
			control_command = CommandMessage()
		
			control_command.enable_taxi = False
			control_command.des_x = 0
			control_command.des_y = 0
			control_command.enable_userCtrl = False
			control_command.enable_helper = False		
			
			command.publish(control_command) # Reset the configuration, canceling every past command
			print('\n	Aborted by the user')
			return
		else:
			if time_left > 0:
				if countdown != time_left:
					print('	{} seconds before aborting'.format(time_left))
				countdown = time_left
			else:
				print('\n	Aborting..')

		#print('\n	Goal reached')

		#print('\n	Aborted by the server: Not reachable')

		#print('\n	Aborted by the client: it has taken too much')
	
		#print('\n	Aborted by the user')
	if feedback == 3:
		print('\n	Goal reached')
	elif feedback == 4:
		print('\n	Aborted by the server: Not reachable')
	elif feedback == 5:
		print('\n	Aborted by the client: it has taken too much')
	else:
		print('\n	Aborted') #?????????????
	sys.stdout.flush()



if __name__ == '__main__':
    main()
