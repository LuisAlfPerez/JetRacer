#!/usr/bin/env python3

from setup.nvidia_racecar import NvidiaRacecar
import rospy
from std_msgs.msg import Int32

def determineSteerValue(message):
	rospy.loginfo("Steering: %d", message.data)
	car.steering = (message.data)
def listener():
	rospy.init_node('control', anonymous=True)
	rospy.Subscriber("referenceDistance", Int32, determineSteerValue)
	r = rospy.Rate(10) 
    while not rospy.is_shutdown:
        r.sleep()
    rospy.spin()
if __name__ == '__main__':
	listener()