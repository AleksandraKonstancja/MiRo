#!/usr/bin/env python

"""
This is a ActionManager class of the MiRo project
"""
import unittest
import random
from time import time
from miro_msgs.msg import platform_control
from miro_constants import miro

class ActionManager:
			
	def __init__(self, isStop=False):
		
		self.isStop = isStop
		
class Command:
	
	def equals(self,o):
		if self.info == o.info:
			return True
	
	def addStop(self):
		self.sequence.append(ActionManager(True))
		
	def __init__(self, info):
		self.info = info
		self.sequence = [ActionManager(),ActionManager(True)]
			
class Robot:
	
	def respond(self,feedback):
		for a in last_command.sequence:
			if a.isStopAction:
				if feedback < 0:
					a = ActionManager()
					break
		
	
	def _init_(self):
		last_command = None
		known_commands = []
		
if __name__ == "__main__":
	
	rob = Robot()
	com1 = Command("redTriangle")
	
			
		
class TestClass(unittest.TestCase):
	
	def test_init(self):
		am = ActionManager()
		for key in am.probabilityDict.keys():
			self.assertTrue(am.probabilityDict[key] == 100.0/6)
			
	def test_highestProb(self):
		am = ActionManager()
		am.probabilityDict["go"] = 50
		self.assertTrue(am.findHighestProb() == "go")
		
	def test_updateProbs(self):
		am = ActionManager()
		am.updateProbs("go", 10)
		prob_sum = 0
		for key in am.probabilityDict.keys():
			prob_sum += am.probabilityDict[key]
		
		#print "sum: " + str(prob_sum)
		self.assertEquals(round(prob_sum,2), 100.0)
		
		prob_sum=0
		am.updateProbs("turnL", 90)
		for key in am.probabilityDict.keys():
			prob_sum += am.probabilityDict[key]
		
		#print "sum: " + str(prob_sum)
		self.assertEquals(round(prob_sum,2), 100.0)
		#print am.probabilityDict
		self.assertEquals(round(am.probabilityDict["turnL"],2), 100.0)
		
		
	def test_performAction(self):
		am = ActionManager()
		print am.probabilityDict
		#am.updateProbs("go", 100)
		am.performAction()
		
			
if __name__ == '__main__':
	#unittest.main()
	import rostest
	rostest.rosrun('test_roslaunch', 'action_manager_test', TestClass)




