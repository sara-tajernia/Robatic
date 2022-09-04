#!/usr/bin/python3

from rand_stu.msg import Stu
from random import randint, seed, choice
from time import time
import rospy

pub_soft = rospy.Publisher('Software', Stu, queue_size=10)
pub_hard = rospy.Publisher('Hardware', Stu, queue_size=10)


def classify(stu):
    	if(stu.departement == "Software"):
    		pub_soft.publish(stu)

    	if(stu.departement == "Hardware"):
    		pub_hard.publish(stu)
    	    
    	    
def split():
	rospy.init_node('splitter', anonymous=True)
	rospy.Subscriber("students", Stu, classify)
	rospy.loginfo('splitting')
	rospy.spin()
	
if __name__ == '__main__':
	try:
		split()
	except rospy.ROSInterruptException:
		pass




