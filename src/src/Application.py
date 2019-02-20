#!/usr/bin/env python

"""
This is a main class of the MiRo project
"""

import rospy
import time
import Robot

class Application:
	
	def loop(self):
		while True:
			if rospy.core.is_shutdown():
				break
			time.sleep(1)
			
	
	def __init__(self):
		
		robot = Robot.Robot()
		

if __name__ == "__main__":

	rospy.init_node("miro_ros_client_py", anonymous=True)
	main = Application()
	main.loop()
    


