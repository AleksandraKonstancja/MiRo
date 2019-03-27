#!/usr/bin/env python
"""
This is a ActionManager class of the MiRo project
"""
import unittest
from Robot import Robot
from Command import Command
from Application import Application
from ActionManager import ActionManager
import cv2
import sys
#sys.modules['rospy'] = mock.Mock()
		
class TestClass(unittest.TestCase):
	
		
	def test_save(self):

		app = Application()
		rob = Robot()
		com = Command("left")
		com.sequence = [ActionManager(),ActionManager(True)]
		rob.known_commands = [com]
		app.robot= rob
		app.save("test2.txt")
		
		
		
			
if __name__ == '__main__':
	#unittest.main()
	import rostest
	rostest.rosrun('test_roslaunch', 'app_test', TestClass)




