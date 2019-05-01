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
		com.action.choice = "go"

		before = com.action.probabilityDict["go"]
		self.assertEquals(round(before,2), 16.67)
		com.updateProbs(1)
		after = com.action.probabilityDict["go"]
		self.assertEquals(round(after,2), 17.67) 
		self.assertTrue(after > before)
		
	def test_command_creation(self):
		
		com = Command("left")
		
		self.assertFalse(com.sequence[0].isStopAction)
		self.assertTrue(com.sequence[1].isStopAction)
		
		
	def test_detection(self):
		
		com = Command("left")
		image = cv2.imread("testImage.jpg")
		
		self.assertEquals(com.colour, "")
		self.assertEquals(com.shape, "")
		
		com.getCommandData(image)
		
		self.assertEquals(com.colour, "red")
		self.assertEquals(com.shape, "triangle")
		
		
		
			
if __name__ == '__main__':
	#unittest.main()
	import rostest
	rostest.rosrun('test_roslaunch', 'command_test', TestClass)




