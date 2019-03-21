#!/usr/bin/env python

"""
This is a main class of the MiRo project
"""

import rospy
import time
import Robot
import cv2

class Application:
	
	def save(self):
		fn = open("test.txt", "w")
		
		for d in self.robot.known_commands:
			fn.write(str(d.toPrint()) + "\n")
			print str(d) + "\n"
			
	
	def loop(self):
		while True:
			if rospy.core.is_shutdown():
				save()
				cv2.destroyAllWindows()
				break
			time.sleep(1)
			
	
	def __init__(self):
		
		self.robot = Robot.Robot()
		

if __name__ == "__main__":

	rospy.init_node("miro_ros_client_py", anonymous=True)
	main = Application()
	main.loop()
    


