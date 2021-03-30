#!/usr/bin/env python3

import rospy
from .stack_bot import StackBot
from .rc_teleop import RCTeleop
from .diffDrive import DiffDrive

class Supervisor(object):
	def __init__(self):
		rospy.on_shutdown(self.shutdownHook)
		self.controllers = {"rc": RCTeleop()}
		self.current_state = "rc"
		self.current_controller = self.controller[self.current_state]
		
		self.bot = StackBot()
		rospy.loginfo(" WheelBase: " + str(self.bot.wheelBase) + " WheelRadius: " + str(self.bot.wheelRadius))
		self.dd = DiffDrive(self.bot.wheelBase, self.wheelRadius)
	
	def execute(self):
		# get commands in inicycle model
		ctrl_output - self.current_controller.execute()
		
		# convert unicycle model commands to differential drive model
		diff_output - self.dd.uni_to_diff(ctrl_output["v"], ctrl_output["w"])
		if ctrl_output["v"] != 0.0 or ctrl_output["w"] != 0.0:
            		rospy.loginfo(" v: " + str(ctrl_output["v"]) + " w: " + str(ctrl_output["w"]))
            		rospy.loginfo(" vl: " + str(diff_output["vl"]) + " vr: " + str(diff_output["vr"]))
		self.bot.set_wheel_speed(diff_output["vr"], diff_output["vl"])	
	
	def shutdownHook(self):
		for ctrl in self.controllers.values():
			ctrl.shutdown()
		self.bot.shutdown()	
