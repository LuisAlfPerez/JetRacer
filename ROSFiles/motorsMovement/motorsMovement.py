#!/usr/bin/env python3

from setup.nvidia_racecar import NvidiaRacecar
import rospy
from std_msgs.msg import Float32

def moveSteer(message):
	car.steering = float(message)/100
	rospy.loginfo("Steering: %s", message)

def moveMotor(message):
	car.throttle = float(message)/100
	rospy.loginfo("Throttle: %s", message)

def listener():
	rospy.init_node('motors', anonymous=True)
	rospy.Subscriber("steering", Float32, moveSteer)
	rospy.Subscriber("motor", Float32, moveMotor)


listener()
car = NvidiaRacecar()
car.steering = 0
car.steering_gain = 0.6
car.steering_offset = 0
car.throttle = 0
car.throttle_gain = 1
