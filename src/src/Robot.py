#!/usr/bin/env python

"""
This is a Robot class of the MiRo project
"""

import rospy
import time
from Command import Command
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from geometry_msgs.msg import Twist
import miro_msgs
from miro_msgs.msg import platform_mics, core_control, platform_sensors, core_state, core_config, platform_control
from miro_constants import miro
from sensor_msgs.msg import Image
import time
from os import system, name 

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
		self.updateMood()
		
		if (self.waitingForFeedback and self.isHappy()):

			for c in self.known_commands:
				if c.equals(self.last_command):
		
					self.last_command.updateProbs( (self.mood) )
					c = self.last_command
			
		
		self.q.tail = round(self.emotion,1)
		self.pub_platform_control.publish(self.q)
		
		self.count += 1
	
	def commandKnown(self, com):
		for c in self.known_commands:
			if com.isCommand() and c.equals(com):
				return True
				
		return False
		
	def findCommand(self,com):
		for c in self.known_commands:
			if c.equals(com):
				return c
		return None
		
		
	def detectCommand(self, image, camera):
		
		com = Command(camera)
		com.getCommandData(image)
			
		if self.commandKnown(com):
			com = self.findCommand(com)
			self.last_command = com
			print("Found command: " + com.toPrint())
			self.q = self.last_command.performAction()
			self.pub_platform_control.publish(self.q)
			q = platform_control()
			print "action finished, waiting for feedback"
			#print "eyelids: " + str(q.eyelid_closure)
			self.waitingForFeedback = True
			cur_time = time.time()
			end_time = cur_time+3
			while cur_time<end_time:
				cur_time = time.time()
			print "stopped waiting for feedback"
			self.waitingForFeedback = False
		elif not self.commandKnown(com) and com.isCommand():
			self.known_commands.append(com)
			last_command = com
		else:
			self.q = platform_control()
			
	def detectLeft( self, object):
		bridge = CvBridge()
		image = bridge.imgmsg_to_cv2(object, "bgr8")
		self.detectCommand(image, "left")
		
	def detectRight( self, object):
		bridge = CvBridge()
		image = bridge.imgmsg_to_cv2(object, "bgr8")
		self.detectCommand(image, "right")
		
			
		
	
	def addCommand(self, c):
		self.known_commands.append(c)
	
	def updateEmotion(self, isTouched):
		
		if ( isTouched and self.emotion < 1 ):
			self.emotion += 0.0005
		elif (not isTouched) and self.emotion > 0:  #used to be just not touched
			self.emotion -= 0.0001
		#print("emotion: " + str(self.emotion))
		
	def updateMood(self):
		
		sync_rate = 50
		time = self.count / sync_rate

		if ( time > 5 ) and self.isHappy():
			self.mood+=0.1
			self.count = 0
			print ("mood: " + str(self.mood) + "time: " + str(time))
		elif (time>5) and (not self.isHappy()) and self.mood>=0.01:
			self.mood-=0.01
			print ("mood: " + str(self.mood) + "time: " + str(self.count))
			self.count = 0
			
	def isHappy(self):
		return (self.emotion > 0)
		
		
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
