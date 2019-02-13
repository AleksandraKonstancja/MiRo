#!/usr/bin/env python
#	@file
#	@section COPYRIGHT
#	Copyright (C) 2018 Consequential Robotics (CqR)
#	
#	@section AUTHOR
#	Consequential Robotics http://consequentialrobotics.com
#	
#	@section LICENSE
#	For a full copy of the license agreement, see LICENSE.MDK in
#	the MDK root directory.
#	
#	Subject to the terms of this Agreement, Consequential Robotics
#	grants to you a limited, non-exclusive, non-transferable license,
#	without right to sub-license, to use MIRO Developer Kit in
#	accordance with this Agreement and any other written agreement
#	with Consequential Robotics. Consequential Robotics does not
#	transfer the title of MIRO Developer Kit to you; the license
#	granted to you is not a sale. This agreement is a binding legal
#	agreement between Consequential Robotics and the purchasers or
#	users of MIRO Developer Kit.
#	
#	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
#	EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
#	OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
#	NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
#	HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
#	WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
#	FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
#	OTHER DEALINGS IN THE SOFTWARE.
#	
#	@section DESCRIPTION
#

################################################################

import rospy
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image,CompressedImage
from geometry_msgs.msg import Twist
from os import system, name 

import miro_msgs
from miro_msgs.msg import platform_config,platform_sensors,platform_state,platform_mics,platform_control,core_state,core_control,core_config,bridge_config,bridge_stream

import math
import numpy
import time
import sys
from miro_constants import miro

################################################################

def fmt(x, f):
    s = ""
    x = bytearray(x)
    for i in range(0, len(x)):
        if not i == 0:
            s = s + ", "
        s = s + f.format(x[i])
    return s

def hex2(x):
    return "{0:#04x}".format(x)

def hex4(x):
    return "{0:#06x}".format(x)

def hex8(x):
    return "{0:#010x}".format(x)

def flt3(x):
    return "{0:.3f}".format(x)

def error(msg):
    print(msg)
    sys.exit(0)

def usage():
    print """
Usage:
    miro_ros_client.py robot=<robot_name>

    Without arguments, this help page is displayed. To run the
    client you must specify at least the option "robot".

Options:
    robot=<robot_name>
        specify the name of the miro robot to connect to,
        which forms the ros base topic "/miro/<robot_name>".
        there is no default, this argument must be specified.
    """
    sys.exit(0)

################################################################
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
	
def clear():
	if not name == 'nt': 
		_ = system('clear')
    	
	
	
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
        
        # ignore until active
        if not self.active:
            return

        # store object
        self.platform_sensors = object

        # send downstream command, ignoring upstream data
        q = platform_control()

        # timing
        sync_rate = 50
       	 # time of the movement
        period = 2 * sync_rate # two seconds per period;
        z = self.count / period
       
        # advance pattern
        if not z == self.z_bak:
            self.z_bak = z
            
            # create body_vel for next pattern segment
            self.body_vel = Twist()
            if self.action in actions.keys():
            	actions[self.action](self)
            else:
            	print "This action doesn't exist" #make it an error
            	

        # publish
        q.body_vel = self.body_vel
        q.body_config[1] = self.body_config[1]
        q.body_config_speed[1] = self.body_config_speed[1]
        q.eyelid_closure = self.eyelid_closure
        q.ear_rotate[0] = self.ear_rotate[0]
        q.ear_rotate[1] = self.ear_rotate[1]
        self.pub_platform_control.publish(q)

        # count
        self.count = self.count + 1
      #  if self.count == 500: #original: 400
      #      self.count = 0
            
        

    def callback_platform_state(self, object):
        
        # ignore until active
        if not self.active:
            return

        # store object
        self.platform_state = object

    def callback_platform_mics(self, object):
        
        # ignore until active
        if not self.active:
            return

        # store object
        self.platform_mics = object
    
	
    def callback_core_state(self, object):
        
        # ignore until active
        if not self.active:
            return

        # store object
        self.core_state = object
        print object.emotion
        time.sleep(2)
        clear()
        
        

    def loop(self):
        while True:
            if rospy.core.is_shutdown():
                break
            time.sleep(1)
            print "tick"
            if rospy.core.is_shutdown():
                break
            time.sleep(1)
            print "tock"

    def __init__(self):
        
        # report
        print("initialising...")
        print(sys.version)

        # default data
        self.platform_sensors = None
        self.platform_state = None
        self.platform_mics = None
        self.core_state = None
        self.eyelid_closure = 0
        self.ear_rotate = [0,0]
        self.body_config = [0,0,0]
        self.body_config_speed = [0,0,0]
        
        self.action_finished = False

        # no arguments gives usage
        if len(sys.argv) == 1:
            usage()

        # options
        self.robot_name = ""
        self.action = ""

        # handle args
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
        self.z_bak = -1
        self.body_vel = None

        # set inactive
        self.active = False

        # topic root
        topic_root = "/miro/" + self.robot_name
        print "topic_root", topic_root
        
        # publish
        self.pub_platform_control = rospy.Publisher(topic_root + "/platform/control",
                    platform_control, queue_size=0)
        self.pub_core_control = rospy.Publisher(topic_root + "/core/control",
                    core_control, queue_size=0)
        self.pub_core_config = rospy.Publisher(topic_root + "/core/config",
                    core_config, queue_size=0)
        self.pub_bridge_config = rospy.Publisher(topic_root + "/bridge/config",
                    bridge_config, queue_size=0)
        self.pub_bridge_stream = rospy.Publisher(topic_root + "/bridge/stream",
                    bridge_stream, queue_size=0)
        self.pub_platform_config = rospy.Publisher(topic_root + "/platform/config",
                    platform_config, queue_size=0)
        self.pub_cosmetic_joints = rospy.Publisher(topic_root + "/control/cosmetic_joints", Float32MultiArray, queue_size = 0) 
      

        # subscribe
        self.sub_sensors = rospy.Subscriber(topic_root + "/platform/sensors",
                platform_sensors, self.callback_platform_sensors)
        self.sub_state = rospy.Subscriber(topic_root + "/platform/state",
                platform_state, self.callback_platform_state)
        self.sub_mics = rospy.Subscriber(topic_root + "/platform/mics",
                platform_mics, self.callback_platform_mics)
        self.sub_core_state = rospy.Subscriber(topic_root + "/core/state",
                core_state, self.callback_core_state)
        
        # set active
        self.active = True

if __name__ == "__main__":
    rospy.init_node("miro_ros_client_py", anonymous=True)
    main = miro_ros_client()
    main.loop()
    


