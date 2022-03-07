#!/usr/bin/env python3
from std_msgs.msg import Int8
import rospy

def publisher():
	publisher = rospy.Publisher('steering', Int8, queue_size=10)
	rospy.init_node('steeringWrite', anonymous=True)
	while not rospy.is_shutdown():
		val = input("Enter steering value (-100 to 100)")
		try:
			publisher.publish(int(val))
		except:
			break 

if __name__ == '__main__':
	try:
		publisher()
	except:
		pass
