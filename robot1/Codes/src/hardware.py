#!/usr/bin/python3

import rospy
from rand_stu.msg import Stu


def printHardwares(stu):
    	rospy.loginfo('\ninfo studente of hardware departement: \n name: \t\t\t{}\n last name: \t\t{}\n age: \t\t\t{}\n departement:\t\t{}\
			\n'.format(stu.name, stu.last_name, stu.age, stu.departement))

    
def hardwares():
    	rospy.init_node('hardware', anonymous=True)
    	rospy.Subscriber("Hardware", Stu, printHardwares)
    	rospy.loginfo('spliting hardware...')
    	rospy.spin()
    	
if __name__ == '__main__':
	try:
		hardwares()
	except rospy.ROSInterruptException:
		pass


