#!/usr/bin/env python3
from std_msgs.msg import Float32
import rospy

def publisher():
	publisher = rospy.Publisher('motor', Float32, queue_size=10)
	rospy.init_node('motorWrite', anonymous=True)
	while not rospy.is_shutdown():
		val = input("Enter motor value (-1 to 1)")
		try:
			publisher.publish(float(val))
		except:
			break 

if __name__ == '__main__':
	try:
		publisher()
	except:
		pass