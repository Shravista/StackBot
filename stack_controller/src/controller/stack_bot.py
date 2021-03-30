#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class StackBot(object):
	def __init__(self):
		self.wheelBase = rospy.get_param("wheelBase",default=0.13)
		self.wheelRadius = rospy.get_param("wheelRadius",default=0.030)
		
		# Wheel min and max no-load velocities in radians per sec
		self.wheel_speed_min = rospy.get_param("wheel_speed/min",default=3.1)
		self.wheel_speed_mid = rospy.get_param("wheel_speed/mid",default=4.4)
		self.wheel_speed_max = rospy.get_param("wheel_speed/max",default=5.48)
		
		self.wheel_min_power = rospy.get_param("wheel_speed/min_power", default = 0.5)
		self.wheel_mid_power = rospy.get_param("wheel_speed/mid_power", default = 0.75)
		self.wheel_max_power = rospy.get_param("wheel_speed/max_power", default = 1.0)
		
		# initializing the topics for wheel_power
		self.cur_Wpow_right = Float32()
		self.cur_Wpow_left = Float32()
		self.cur_Wpow_left.data = 0.0
		self.cur_Wpow_right.data = 0.0
		
		self.WRight_pub = rospy.Publisher("/wheel_power_right", Float32, queue_size=10)
		self.WLeft_pub = rospy.Publisher("/wheel_power_left", Float32, queue_size=10)
		
		self.WRight_pub.publish(self.cur_Wpow_right)
		self.WLeft_pub.publish(self.cur_Wpow_left)
	
	def shutdown(self):
		rospy.loginfo(rospy.get_caller_id() + "StackBot shutting down ...")
		self.cur_Wpow_left.data = 0.0
		self.cur_Wpow_right.data = 0.0
		self.WRight_pub.publish(self.cur_Wpow_right)
		self.WLeft_pub.publish(self.cur_Wpow_left)
	
	def velocity_to_power(self,v):
		modV = abs(v)
		
		# If velocity is below minimum velocity turnable by PWM, then
        	# just set to zero since the wheels won't spin anyway
		if av < self.wheel_speed_min:
			return 0.0
		a = b = a_pow = b_pow = None
		nnn = None
		if av >= self.wheel_speed_min and av < self.wheel_speed_mid:
			a = self.wheel_speed_min
			a_pow = self.wheel_min_power
			b = self.wheel_speed_mid
			b_pow = self.wheel_mid_power
		elif av >= self.wheel_speed_mid and av < self.wheel_speed_max:
			a = self.wheel_speed_mid
			a_pow = self.wheel_mid_power
			b = self.wheel_speed_max
			b_pow = self.wheel_max_power
		
		# linearly interpolate a and b
		nnn = ((av - a)/(b -a))
		wheel_power = ((nnn * (b_pow - a_pow)) + a_pow)
		if False:
			rospy.loginfo(rospy.get_caller_id() + ": " + str(a) + "," + str(b) +", " + str(a_pow) + "," + str(b_pow))
			rospy.loginfo(rospy.get_caller_id() + " av: " + str(av))
			rospy.loginfo(rospy.get_caller_id() + " nnn: " + str(nnn))
			rospy.loginfo(rospy.get_caller_id() + " wheel_power: " + str(wheel_power))
		
		assert(wheel_power <= 1.0)
		assert(wheel_power >= 0.0)
		# Negate if necessary
		if v < 0:
			wheel_power *= -1.0

		return wheel_power

	def set_wheel_speed(self, vr, vl):
		# clamp the wheel speeds to actuator limits
		vr = max(min(vr, self.wheel_speed_max), self.wheel_speed_max * -1.0)
		vl = max(min(vl, self.wheel_speed_max), self.wheel_speed_max * -1.0)

		# convert power norms

		self.cur_Wpow_left.data = self.velocity_to_power(vl)
		self.cur_Wpow_right.data = self.velocity_to_power(vr)

		if self.cur_Wpow_left.data !=0.0 or self.cur_Wpow_right.data !=0.0:
			rospy.loginfo(rospy.get_caller_id() + "right power: "+ str(self.cur_Wpow_right) + " left power: " + str(self.cur_Wpow_left))
		self.WRight_pub.publish(self.cur_Wpow_right)
		self.WLeft_pub.publish(self.cur_Wpow_left)
