#!/usr/bin/env python

"""
This is a main class of the MiRo project
"""

import rospy
import time
import Robot
import cv2

class Application:
	
	def save(self, name):
		fn = open(name, "w")
		
		for d in self.robot.known_commands:
			
			actions = ""
			
			for a in d.sequence:
				actions += ("\n" + str(a.probabilityDict))
				
			fn.write(str(d.toPrint()) + " " + str(actions) + "\n")
			#print str(d) + "\n"
			
	
	def loop(self):
		while True:
			answer = input("Do you want to exit?")
			if answer == 'y':
				#rospy.core.is_shutdown():
				self.save("test.txt")
				print "bye"
				cv2.destroyAllWindows()
				break
			time.sleep(1)
			
	
	def __init__(self):
		
		self.robot = Robot.Robot()
		

if __name__ == "__main__":

	rospy.init_node("miro_ros_client_py", anonymous=True)
	main = Application()
	main.loop()
    


