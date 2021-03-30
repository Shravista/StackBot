#!/usr/bin/env python3

import rospy

from controller.supervisor import Supervisor

def main():
	rospy.init_node("move_stackBot", anonymous=False)
	supervisor = Supervisor()
	
	rate = rospy.Rate(2)
	
	while not rospy.is_shutdown():
		supervisor.execute
