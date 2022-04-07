#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32

publisherMotor = None
publisherSteering = None

def determineSteerValue(message):
	global publisherMotor
	global publisherSteering
	rospy.loginfo("Steering: %d", message.data)
	steering = message.data / 25
	motor = 0.3
	if steering > 1:
		steering = 1
	if steering < -1:
		steering = -1
	if motor > 1:
		motor = 1
	publisherSteering.publish(float(steering))
	publisherMotor.publish(float(motor))
	
def listener():
	global publisherMotor
	global publisherSteering
	publisherMotor = rospy.Publisher('motor', Float32, queue_size=1)
	publisherSteering = rospy.Publisher('steering', Float32, queue_size=1)
	rospy.init_node('control', anonymous=True)
	rospy.Subscriber("referenceDistance", Int32, determineSteerValue)
	r = rospy.Rate(10) 
	while not rospy.is_shutdown:
	    r.sleep()
	rospy.spin()
if __name__ == '__main__':
	listener()