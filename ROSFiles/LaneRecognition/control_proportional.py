#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Int32
from std_msgs.msg import Float32

publisherMotor = None
publisherSteering = None
currentError = None
latestError = None

def determineSteerValue(message):
	global publisherMotor
	global publisherSteering
	global currentError
	global latestError
	
	sleep_time = 0.1

	currentError = message.data
	k_proportional = 1/100
	
	steering = k_proportional*currentError
	motor = -0.20
	
	if steering > 1:
		steering = 1
	if steering < -1:
		steering = -1
	
	rospy.loginfo("Steering: %d", steering)
	publisherSteering.publish(float(steering))
	
	extraMotor = 0.075
	motorSteady = -0.15
	
	if abs(steering) > 0.75:
		publisherMotor.publish(float(motor+extraMotor))
		time.sleep(sleep_time)
		publisherMotor.publish(float(motorSteady))
	else:
		publisherMotor.publish(float(motor))
		time.sleep(sleep_time)
		publisherMotor.publish(float(motorSteady))
	
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