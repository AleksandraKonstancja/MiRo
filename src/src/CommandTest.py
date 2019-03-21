#!/usr/bin/env python
"""
This is a ActionManager class of the MiRo project
"""
import unittest
from Robot import Robot
from Command import Command
import cv2
import sys
#sys.modules['rospy'] = mock.Mock()
		
class TestClass(unittest.TestCase):
	
		
	def test_update_probs(self):

		com = Command("left")
		com.colour = "red"
		com.shape = "triangle"
		com.action.last_action = "go"

		before = com.action.probabilityDict["go"]
		self.assertEquals(round(before,2), 16.67)
		com.updateProbs(1)
		after = com.action.probabilityDict["go"]
		self.assertEquals(round(after,2), 17.67) 
		self.assertTrue(after > before)
		
		
		
			
if __name__ == '__main__':
	#unittest.main()
	import rostest
	rostest.rosrun('test_roslaunch', 'command_test', TestClass)




