#!/usr/bin/env python

"""
This is a Robot class of the MiRo project
"""

import rospy
import time
from Command import Command
from ActionManager import ActionManager
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from geometry_msgs.msg import Twist
import miro_msgs
from miro_msgs.msg import platform_mics, core_control, platform_sensors, core_state, core_config, platform_control
from miro_constants import miro
from sensor_msgs.msg import Image
import time
from os import system, name 
import thread

"""
Formating function copied from miro_ros_client.py
"""
def fmt(x, f):
    s = ""
    x = bytearray(x)
    for i in range(0, len(x)):
        if not i == 0:
            s = s + ", "
        s = s + f.format(x[i])
    return s

class Robot:
	
	def callback_platform_sensors(self, object):
		
		if not self.active:
			return
			
		self.platform_sensors = object
		
		#q = platform_control()
		
		self.updateEmotion(self.detectTouch())
		self.updateMood(500)
		#self.updateProbs()
		"""if (self.waitingForFeedback and self.isHappy()):

			for c in self.known_commands:
				if c.equals(self.last_command):
		
					self.last_command.updateProbs( (self.mood) )
					c = self.last_command"""
			
		
		self.q.tail = round(self.emotion,1)
		self.pub_platform_control.publish(self.q)
		
		self.count += 1
	
	"""
	If robot is rewarded by touch updates probability of last performed action 
	for last seen command based on the mood state.
	"""
	def updateProbs(self):
		if (self.waitingForFeedback and self.isHappy()):

			for c in self.known_commands:
				if c.equals(self.last_command):
		
					self.last_command.updateProbs( (self.mood) )
					c = self.last_command
					
	def updateProbs2(self, feedback):
		
		for c in self.known_commands:
			if c.equals(self.last_command):
				self.last_command.updateProbs2( (feedback*(1+self.mood)) )
				c = self.last_command
					
	"""
	Checks if the command has already been learned or is seen for the first time.
	Returns true if command is known, false if it is not
	"""
	def commandKnown(self, com):
		for c in self.known_commands:
			if com.isCommand() and c.equals(com):
				return True
				
		return False
	
	"""
	Finds and returns the command in the list of learned commands.
	Returns command from the list if it is found, or None if it does not exist
	in the list.
	"""
	def findCommand(self,com):
		for c in self.known_commands:
			if c.equals(com):
				return c
		return None
		
	"""
	Detects command on the given image and responds to it.
	"""
	def detectCommand(self, image, camera):
		
		com = Command(camera)
		com.getCommandData(image)
		
		if self.last_command == None:
			self.state = "Looking for commands"
			self.responded = False
		
			if self.commandKnown(com):
				com = self.findCommand(com)
				self.last_command = com
			elif not self.commandKnown(com) and com.isCommand():
				com.saveImage()
				self.known_commands.append(com)
				self.last_command = com
			else:
				self.q = platform_control()
		elif not self.responded:
			thread.start_new_thread(self.respondToCommand, ())
			self.responded = True
				
	"""
	Performs an action based on its probability and waits for feedback for 3 seconds.
	"""
	def respondToCommand(self):
		
		print("Found command: " + self.last_command.toPrint())
		#self.last_command.performAction()
		
		"""self.pub_platform_control.publish(self.q)
		q = platform_control()"""
		
		for i in range(len(self.last_command.sequence)):
			self.q = self.last_command.sequence[i].performAction()
			self.state = "Responding to " + self.last_command.toPrint() + " with " + self.last_command.sequence[i].choice
			#if (not a.isFullyLearned()):
			feedback = self.collectFeedback()
			if self.last_command.sequence[i].isStopAction:
				if feedback < 0:
					self.last_command.sequence[i] = ActionManager()
				break
			else:
				self.last_command.sequence[i].updateProbs(feedback)

		if not self.last_command.sequence[-1].isStopAction:
			self.last_command.addStopAction()
			
		for c in self.known_commands:
			if c.equals(self.last_command):
				c = self.last_command
			
		self.last_command = None
		
	def collectFeedback(self):
		
		#self.state = "action finished, waiting for feedback"
		print ("action finished, waiting for feedback")
		#self.waitingForFeedback = True
		feedback = -1
		cur_time = time.time()
		end_time = cur_time+5
		while cur_time<end_time:
			cur_time = time.time()
			if self.emotion == 1:
				feedback = 1
		print ("stopped waiting for feedback")
		#self.waitingForFeedback = False
		return feedback
		
	"""
	Prepares image from left camera for detection and detects command.
	"""
	def detectLeft( self, object):
		bridge = CvBridge()
		image = bridge.imgmsg_to_cv2(object, "bgr8")
		self.leftCam = image
		#thread.start_new_thread( self.detectCommand, ( image, "left"))
		self.detectCommand(image, "left")
		
		
	"""
	Prepares image from right camera for detection and detects command.
	"""
	def detectRight( self, object):
		bridge = CvBridge()
		image = bridge.imgmsg_to_cv2(object, "bgr8")
		self.rightCam = image
		#thread.start_new_thread( self.detectCommand, ( image, "right"))
		self.detectCommand(image, "right")
		
	"""
	Adds command to the list of known commands
	"""
	def addCommand(self, c):
		self.known_commands.append(c)
	
	"""
	Updates emotion based on whether robot is touched or not
	"""
	def updateEmotion(self, isTouched):
		
		if isTouched:
			self.emotion = 1.0
		else:
			self.emotion = 0.0
			
		self.emotions.append(self.emotion)
		
	"""
	Updates mood based on average emotion over last emotions. 
	emotionNumber - number of emotions used to calculate the average
	"""
	def updateMood(self, emotionNumber):
		
		#emotionNumber = 250 # collected for 5 seconds, 50 readings per sec
		
		if len(self.emotions) == emotionNumber:
			avrg = sum(self.emotions)/emotionNumber
			if avrg > 0.5:
				self.mood+=0.2
			elif avrg <= 0.5 and self.mood >=0.1:
				self.mood-=0.1
			self.emotions = []
			print ("mood: " + str(self.mood) + " avrg: " + str(avrg))
		
			
		
		"""sync_rate = 50
		time = self.count / sync_rate

		if ( time > 5 ) and self.isHappy():
			self.mood+=0.1
			self.count = 0
			print ("mood: " + str(self.mood) + "time: " + str(time))
		elif (time>5) and (not self.isHappy()) and self.mood>=0.01:
			self.mood-=0.01
			print ("mood: " + str(self.mood) + "time: " + str(self.count))
			self.count = 0"""
			
	def isHappy(self):
		return (self.emotion > 0)
		
	"""
	Detects whether the robot is touched or not.
	"""
	def detectTouch(self):
		
		for sensor in self.platform_sensors.touch_body:
			if fmt(sensor, '{0:.0f}') == '1':
				return True
		for sensor in self.platform_sensors.touch_head:
			if fmt(sensor, '{0:.0f}') == '1':
				return True
		return False
	

	def __init__(self):
		self.platform_sensors = None
		self.mics = None
		self.known_commands = []
		self.last_emotion = 0.0
		self.emotion = 0.0
		self.mood = 0.0
		self.count = 0
		self.waitingForFeedback = False
		self.actionInProgress = False
		self.last_command = None
		self.emotions = []
		self.actionFinished = True
		self.responded = False
		self.state = ""
		self.leftCam = None
		self.rightCam = None
		
		self.eyelid_closure = 0
		self.q = platform_control()

		
		#publish
		topic_root = "/miro/rob01"
		self.pub_platform_control = rospy.Publisher(topic_root + "/platform/control", platform_control, queue_size=0)
        
		#subscribe
		self.sub_sensors = rospy.Subscriber(topic_root + "/platform/sensors", platform_sensors, self.callback_platform_sensors)
		self.sub_caml = rospy.Subscriber(topic_root + "/platform/caml", Image, self.detectLeft,queue_size=1)
		self.sub_camr = rospy.Subscriber(topic_root + "/platform/camr", Image, self.detectRight,queue_size=1)
		self.active = True
