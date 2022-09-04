#!/usr/bin/python3

import rospy
from rand_stu.msg import Stu


def printSoftwares(stu):
    	rospy.loginfo('\ninfo studente of hardware departement: \n name: \t\t\t{}\n last name: \t\t{}\n age: \t\t\t{}\n departement:\t\t{}\
			\n'.format(stu.name, stu.last_name, stu.age, stu.departement))

    
def softwares():
    	rospy.init_node('software', anonymous=True)
    	rospy.Subscriber("Software", Stu, printSoftwares)
    	rospy.loginfo('spliting software...')
    	rospy.spin()
    	
if __name__ == '__main__':
	try:
		softwares()
	except rospy.ROSInterruptException:
		pass



