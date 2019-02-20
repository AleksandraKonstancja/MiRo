#!/usr/bin/env python

"""
This is a simple program that given action as a command line argument will
make MiRo execute just this action and exit. Possible actions are:
- go ( goes forward for 200mm )
- turnL ( turns 90 degreed left )
- turnR ( turns 90 degrees right )
- eyes ( closes its eyes if they're open, opens if they're closed )
- ears ( rotates ears )
- headDown ( leans head down )
"""

import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import miro_msgs
from miro_msgs.msg import platform_control, platform_sensors
from miro_constants import miro

from os import system, name 

import time
import sys



def error(msg):
    print(msg)
    sys.exit(0)

def printInstructions():
    print """
Usage:
    actions.py robot=<robot_name> action=<action_type>
	
	Options:
		robot=<robot_name>
			to run the client specify at least this option - the name
			of the miro robot to connect to
		action=<action_type>
			specify action to be performed by robot, available actions are:
			turnR, turnL, eyes, ears, go, headDown
    """
    sys.exit(0)


def getTimeOfAction(self):
	sync_rate = 50
	period = 2 * sync_rate # two seconds per period;
	z = self.count / period
	return z

def turnLeft(self):
	if getTimeOfAction(self) == 0:
		print "turn left"
		self.body_vel.angular.z = +0.7854

def turnRight(self):
	if getTimeOfAction(self) == 0:
		print "turn right"
		self.body_vel.angular.z = -0.7854

def go(self):
	if getTimeOfAction(self) == 0:
		self.body_vel.linear.x = 200

def eyes(self):
	if not self.action_finished:
		self.eyelid_closure = 1 - self.eyelid_closure
		self.action_finished = True
		
def headDown(self):
	self.body_config[1] = miro.MIRO_LIFT_MAX_RAD - self.body_config[1]
	self.body_config_speed[1] = miro.MIRO_P2U_W_LEAN_SPEED_INF
	# 1 for lift, 2 for yaw, 3 for pitch

def ears(self):
	self.ear_rotate[0] = 1 - self.ear_rotate[0]
	self.ear_rotate[1] = 1 - self.ear_rotate[1]    	
	
	
actions = {
"turnL" : turnLeft,
"turnR" : turnRight,
"go" : go,
"eyes" : eyes,
"headDown" : headDown,
"ears" : ears
}
	

class miro_ros_client:

    def callback_platform_sensors(self, object):
        
        if not self.active:
            return

        self.platform_sensors = object

        q = platform_control()

        # Miro runs at 50Hz
        sync_rate = 50
       	 # time of the movement
        period = 2 * sync_rate # 2 seconds
        # self counts increases 50 times a sec so after 1 sec z = 50 / 100 = 0 
        cur_time = self.count / period

        if not cur_time == self.last_time:
            self.last_time = cur_time   
            # create body_vel for next pattern segment
            self.body_vel = Twist()
            if self.action in actions.keys():
            	actions[self.action](self)
            
        # publish
        q.body_vel = self.body_vel
        q.body_config[1] = self.body_config[1]
        q.body_config_speed[1] = self.body_config_speed[1]
        q.eyelid_closure = self.eyelid_closure
        q.ear_rotate[0] = self.ear_rotate[0]
        q.ear_rotate[1] = self.ear_rotate[1]
        self.pub_platform_control.publish(q)

        self.count = self.count + 1
        #print str(self.count) +"\n"
        
        
    def loop(self):
        while True:
            if rospy.core.is_shutdown():
                break
            time.sleep(1)

    def __init__(self):

        # default data
        self.platform_sensors = None
        self.eyelid_closure = 0
        self.ear_rotate = [0,0]
        self.body_config = [0,0,0]
        self.body_config_speed = [0,0,0] 
        self.action_finished = False

        if len(sys.argv) == 1:
            printInstructions()

        # argument options
        self.robot_name = ""
        self.action = ""

        # handle arguments
        for arg in sys.argv[1:]:
            f = arg.find('=')
            if f == -1:
                key = arg
                val = ""
            else:
                key = arg[:f]
                val = arg[f+1:]
            if key == "robot":
                self.robot_name = val
            elif key == "action":
                self.action = val
            else:
                error("argument not recognised \"" + arg + "\"")

        # check we got at least one
        if len(self.robot_name) == 0:
            error("argument \"robot\" must be specified")
        
        # pattern
        self.count = 0
        self.last_time = -1
        self.body_vel = None

        # set inactive
        self.active = False
        
        # publish
        topic_root = "/miro/" + self.robot_name
        self.pub_platform_control = rospy.Publisher(topic_root + "/platform/control",
                    platform_control, queue_size=0)
        
        # subscribe
        self.sub_sensors = rospy.Subscriber(topic_root + "/platform/sensors",
                platform_sensors, self.callback_platform_sensors)
        
        self.active = True

if __name__ == "__main__":
    rospy.init_node("miro_ros_client_py", anonymous=True)
    main = miro_ros_client()
    main.loop()
    


