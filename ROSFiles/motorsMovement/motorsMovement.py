#!/usr/bin/env python3

from setup.nvidia_racecar import NvidiaRacecar
import rospy
from std_msgs.msg import Float32

def moveSteer(message):
	rospy.loginfo("Steering: %d", message.data)
	car.steering = (message.data)

def moveMotor(message):
	car.throttle = (message.data)
	rospy.loginfo("Throttle: %d", message.data)

def listener():
	rospy.init_node('motors', anonymous=True)
	rospy.Subscriber("steering", Float32, moveSteer)
	rospy.Subscriber("motor", Float32, moveMotor)
	rospy.spin()
	rospy.loginfo("Iniciando3")	

car = NvidiaRacecar()
car.steering = 0
car.steering_gain = 0.6
car.steering_offset = 0
car.throttle = 0
car.throttle_gain = 1
listener()
