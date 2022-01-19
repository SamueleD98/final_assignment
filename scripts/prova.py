#! /usr/bin/env python

import rospy
import selectors
import sys
  
def main():
	rospy.init_node('prova')

	rate = rospy.Rate(20)
    
	while not rospy.is_shutdown():

		sel = selectors.DefaultSelector()
		sel.register(sys.stdin, selectors.EVENT_READ)
		print("Enter passcode: ", end='')
		sys.stdout.flush()
		pairs = sel.select(timeout=5)
		if pairs:
			passcode = sys.stdin.readline().strip()
			print('you entered:', passcode)
		else:
			print('\ntimed out')
			
		rate.sleep()
  
if __name__ == '__main__':
	main()
    
    
	
