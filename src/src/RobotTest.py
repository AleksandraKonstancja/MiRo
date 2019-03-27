#!/usr/bin/python2.7
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

		"""rob.updateProbs2(1)
		after = rob.known_commands[0].action.probabilityDict["go"]
		self.assertEquals(round(after,2), 15.29) 
		
		rob.updateProbs2(-1)
		after = rob.known_commands[0].action.probabilityDict["go"]
		self.assertEquals(round(after,2), 14.29) 
		
		rob.mood = 1
		rob.updateProbs2(1)
		after = rob.known_commands[0].action.probabilityDict["go"]
		self.assertEquals(round(after,2), 16.29) """
		

	def test_update_mood(self):
		
		rob = Robot()
		rob.updateMood(5)
		self.assertEquals(rob.mood, 0)
		for i in range(5):
			rob.updateEmotion(True)
			
		rob.updateMood(5)
		self.assertEquals(rob.mood, 0.2)
		
	def test_feedback(self):
		
		"""rob = Robot()
		rob.emotion = 0
		feedback = rob.collectFeedback()
		self.assertEquals(feedback, -1)
		rob.emotion = 1
		feedback = rob.collectFeedback()
		self.assertEquals(feedback, 1)"""
		
	
	def test_sequence(self):
		
		rob = Robot()
		rob.mood = 0
		rob.emotion = 1
		
		com = Command("left")
		rob.known_commands = [com]
		rob.last_command = com
		
		#Robot.collectFeedback = Mock(return_value=-1)
		
		"""before = rob.known_commands[0].sequence[0].probabilityDict["go"]
		rob.respondToCommand()
		after = rob.known_commands[0].sequence[0].probabilityDict["go"]
		self.assertNotEquals(before, after)
		
		rob.last_command = com"""
		rob.emotion = 0
		#import pdb; pdb.set_trace()
		rob.respondToCommand()
		after = rob.known_commands[0].sequence[1].isStopAction
		self.assertFalse(after)
		
		#com1 = rob.known_commands[0]
		
		#self.assertNotEquals(com1, com)
		self.assertEqual(len(rob.known_commands[0].sequence),3)
		#self.assertFalse(com.sequence[1].isStopAction())"""
		
		#action = rob.last_command.sequence[0].choice
		#self.assertEquals(action"""
		
		
	
		
		
		
		
			
if __name__ == '__main__':
	unittest.main()
	#import rostest
	#rostest.rosrun('test_roslaunch', 'robot_test', TestClass)




