#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from .controller import Controller

class RCTeleop(Controller):
	def __init__(self):
		rospy.loginfo(rospy.get_caller_id() +"RCTeleop initialized")
		rospy.Subscriber("/cmd_vel", Twist, self.callback)
	
		self.v = 0 
		self.w = 0
		
	def execute(self):
		rospy.loginfo(rospy.get_caller_id() + "RCTeleop Execute")
		output = {"v": self.v, "w":self.w}
		return output
	
	def shutdown(self):
		rospy.loginfo(rospy.get_caller_id() + "RCTeleop Shuttingdown")
	
	def callback(self,data):
		rospy.loginfo(": Linear.x: %f -- Angular.z: %f", data.linear.x, data.angular.z)
		self.v = data.linear.x
		self.w = data.angular.z
	
	
