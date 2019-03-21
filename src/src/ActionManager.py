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
	
	"""
	Updates last action by the learning rate and recalculates remaining 
	probabilities depending on previous contribution to the total ( 100 )
	of each action.
	learn_rate - how much should action be affected
	"""
	def updateProbs(self, learn_rate):
		action = self.last_action
		prob_sum = learn_rate + self.probabilityDict[action]
		
		# make sure max probability is 100
		if prob_sum > 100.0:
			learn_rate = 100 - self.probabilityDict[action]
		
		prev_rest = 100 - self.probabilityDict[action]
		
		# change probabilities proportionally to previous values
		self.probabilityDict[action] += learn_rate
		rest = 100 - self.probabilityDict[action]
		
		for key in self.probabilityDict.keys():
			if not key == action:
				rest_rate = rest / prev_rest
				self.probabilityDict[key] *= rest_rate
			
		print self.probabilityDict
	
	"""
	Performs an action based on its probability
	"""
	def performAction(self):
		total_prob = 0
		randomNum = random.randint(1,101)
		#print "random: " + str(randomNum)
		
		for key in self.probabilityDict.keys():
			total_prob += self.probabilityDict[key]
			if randomNum < total_prob:
				self.actionDict[key]()
				break
		return self.q
				
	"""
	Performs most likely action
	"""
	def performMostLikelyAction(self):
		self.actionDict[findHighestProb()]
		return
		
	
	"""
	Finds an action with highest probability of being performed
	"""	
	def findHighestProb(self):
		return max(self.probabilityDict, key=self.probabilityDict.get)
	
	"""
	Set of functions responsible for interacting with robot's actuators
	based on action chosen to be performed.
	"""
	def turnLeft(self):
		self.q.body_vel.angular.z = +0.5235 #0.7854
		self.last_action = "turnL"
		print "left"
	def turnRight(self):
		self.q.body_vel.angular.z = -0.5235 #0.7854
		self.last_action = "turnR"
		print "right"
	def go(self):
		self.q.body_vel.linear.x = 100
		self.last_action = "go"
		print "go"
	def eyes(self):
		self.q.eyelid_closure = 1
		self.last_action = "eyes"
		print "eyes"
	def headDown(self):
		self.q.body_config[1] = miro.MIRO_LIFT_MAX_RAD
		self.last_action = "headDown"
		print "hDown"
	def ears(self):
		self.q.ear_rotate[0] = 1 
		self.q.ear_rotate[1] = 1 
		self.last_action = "ears"
		print "ears"
			
	def __init__(self):
		
		self.last_action = ""
		
		self.q = platform_control()
		
		self.actionDict = {
			"turnL" : self.turnLeft,
			"turnR" : self.turnRight,
			"go" : self.go,
			"eyes" : self.eyes,
			"headDown" : self.headDown,
			"ears" : self.ears,
		}
		
		self.probabilityDict = {}
		
		
		self.actionNumber = len(self.actionDict)
		initial_prob = 100.0 / self.actionNumber
		
		for key in self.actionDict.keys():
			self.probabilityDict[key] = initial_prob
			
			
		
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




