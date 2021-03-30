#!/usr/bin/env python3

import rospy

class DiffDrive(object):
	def __init__(self,wheelBase, wheelRadius):
		self.wheelBase = wheelBase
		self.wheelRadius = wheelRadius
	def uni_to_diff (self, v, w):
		# return m/sec wheel velocities
		
		L = self.wheelBase
		R = self.wheelRadius
		
		vr = ((2.0 * v) + (w * L))/(2.0 * R)
		vl = ((2.0 * v) + (-1.0 * w * L)) / (2.0 * R)
		
		return {"vl": vl, "vr":vr}
