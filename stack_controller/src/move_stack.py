#!/usr/bin/env python3

import rospy

from controller.supervisor import Supervisor

def main():
	rospy.init_node("move_stackBot", anonymous=False)
	supervisor = Supervisor()	
	rate = rospy.Rate(2)
	rospy.loginfo("I am here")
	while not rospy.is_shutdown():
<<<<<<< HEAD
            rospy.loginfo("I am now here")
            supervisor.execute()

if __name__ == "__main__":
    main()
=======
		supervisor.execute()
>>>>>>> d326fc6f8489b8f3ca07fccecd421f2761208e9e
