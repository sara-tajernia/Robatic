#!/usr/bin/python3

from rand_stu.msg import Stu
from random import randint, seed, choice
from time import time
import rospy


def randName():
    	global names
    	return choice(names)

def randLName():
    	global lName
    	return choice(lNames)


def randAge():
    	return randint(20,70)


def randDepartement():
    	global departements
    	return choice(departements)


def randStudent():
    	seed(time())
    	student = Stu()
    	student.name = randName()
    	student.age = randAge()

    	student.last_name = randLName()
    	student.departement = randDepartement()

    	return student
    
    
def student_request():
	'''Node Talker'''
	student = Stu()
	pub = rospy.Publisher('students', Stu, queue_size=10)
	rospy.init_node('random_student', anonymous=True)
	rate = rospy.Rate(1) # 1hz -> 1sec
	
	while not rospy.is_shutdown():
		'''for i in range(3):'''
		student = randStudent()
		pub.publish(student)
		rate.sleep()






names = [
    'Ali',
    "Mohammad",
    "Fatemeh",
    "Amir",
    "Reza",
    "Sahar",
    "Aref",
    "Aria",
    "Ahmad",
    "Akbar",
    "Mohammad Reza",
    "Amir Hosein",
    "Saman",
    "Mohsen",
    "Radin",
    "Maryam",
    "Javad",
    "Ramin",
    "Soroush",
    "Farhad",
    "Siamak",
    "Mehran",
    "Karim",
]

lNames = [
    'Akbari',
    'Hashemi',
    "Ghasemi",
    "Hoseini",
    "Eslami",
    "Kazemi",
    "Kashfi",
    "Shahi",
    "Sheikhi",
    "Kabiri",
    "Majidi",
    "Karimi",
    "Ghafori",
    "Pormokhber",
    "Ansari",
    "Modiri",
    "Fallah",
    "Ansarifard",
]


departements= [
    "Software",
    "Hardware",
]
	
