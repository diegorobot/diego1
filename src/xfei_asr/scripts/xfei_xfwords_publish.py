#!/usr/bin/env python
import rospy
from std_msgs.msg import String

if __name__=="__main__":	
	pub = rospy.Publisher('xfwords', String, queue_size = 0)
	rospy.init_node('xfei_xfwords_publish', anonymous=True)
        rate = rospy.Rate(1) # 10hz
	try:		
	    while not rospy.is_shutdown():
	        input_str = raw_input("Please input your words:\n")
		pub.publish(input_str)
	    	rate.sleep()
	except rospy.ROSInterruptException:
	    pass
