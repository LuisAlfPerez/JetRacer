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
	latestError = currentError
	currentError = message.data
	time = 0.5
	k_proportional = 1/100
	k_derivative = 1/150	
	derivative = (currentError - latestError)/time
	steering = k_proportional*currentError + k_derivative * derivative
	motor = -0.2
	if steering > 1:
		steering = 1
	if steering < -1:
		steering = -1
	rospy.loginfo("Steering: %d", steering)
	publisherSteering.publish(float(steering))
	extraMotor = 0.075
	motorSteady = -0.12
	if abs(steering) > 0.75:
		publisherMotor.publish(float(motor+extraMotor))
		time.sleep(0.1)
		publisherMotor.publish(float(motorSteady))
	else:
		publisherMotor.publish(float(motor))
		time.sleep(0.1)
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