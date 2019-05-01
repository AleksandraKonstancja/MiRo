#!/usr/bin/env python
"""
This is a ActionManager class of the MiRo project
"""
import unittest
from Robot import Robot
from Command import Command
import cv2
import sys
#import mock
#sys.modules['rospy'] = mock.Mock()
		
class TestClass(unittest.TestCase):
	
	def test_detect_command(self):
		
		rob = Robot()
		image = cv2.imread("redTriangle.png")
		
		self.assertEquals(len(rob.known_commands),0)
		
		rob.detectCommand(image,"left")
		self.assertNotEquals(len(rob.known_commands),0)
		self.assertNotEquals(rob.last_command, None)
		com = Command("left")
		com.colour = "red"
		com.shape = "triangle"
		self.assertTrue(rob.last_command.equals(com))
		
	def test_update_probs(self):
		
		rob = Robot()
		rob.mood = 0
		com = Command("left")
		com.colour = "red"
		com.shape = "triangle"
		com.action.choice = "go"
		rob.known_commands = [com]
		com2 = com
		rob.last_command = com2
		self.assertEquals(rob.known_commands[0].action.choice, "go")
		
		before = rob.known_commands[0].action.probabilityDict["go"]
		self.assertEquals(round(before,2), 16.67)

		rob.updateProbs2(1)
		after = rob.known_commands[0].action.probabilityDict["go"]
		self.assertEquals(round(after,2), 15.29) 
		
		rob.updateProbs2(-1)
		after = rob.known_commands[0].action.probabilityDict["go"]
		self.assertEquals(round(after,2), 14.29) 
		
		rob.mood = 1
		rob.updateProbs2(1)
		after = rob.known_commands[0].action.probabilityDict["go"]
		self.assertEquals(round(after,2), 16.29) 
		

	def test_update_mood(self):
		
		rob = Robot()
		rob.updateMood(5)
		self.assertEquals(rob.mood, 0)
		for i in range(5):
			rob.updateEmotion(True)
			
		rob.updateMood(5)
		self.assertEquals(rob.mood, 0.1)
		
		
	
		
		
	
		
		
		
		
			
if __name__ == '__main__':
	unittest.main()
	#import rostest
	#rostest.rosrun('test_roslaunch', 'robot_test', TestClass)




