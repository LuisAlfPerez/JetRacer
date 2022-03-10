#!/usr/bin/env python3
from std_msgs.msg import Float32
import rospy

def publisher():
	publisher = rospy.Publisher('steering', Float32, queue_size=10)
	rospy.init_node('steeringWrite', anonymous=True)
	while not rospy.is_shutdown():
		val = input("Enter steering value (-1 to 1)")
		try:
			publisher.publish(float(val))
		except:
			break 

if __name__ == '__main__':
	try:
		publisher()
	except:
		pass
