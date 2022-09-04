#!/usr/bin/python3

from cmath import sqrt
from multiprocessing import set_forkserver_preload
import rospy, tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import radians
import numpy as np
import matplotlib.pyplot as plt

class Controller:
    def __init__(self) -> None:
        rospy.init_node("controller", anonymous=False)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)

        # getting specified parameters
        self.linear_speed = rospy.get_param("/controller/linear_speed") # m/s
        self.angular_speed = rospy.get_param("/controller/angular_speed") # rad/s
        self.goal_angle = radians(rospy.get_param("/controller/goal_angle")) # rad
        self.stop_distance = rospy.get_param("/controller/stop_distance") # m
        self.epsilon = rospy.get_param("/controller/epsilon")
        self.length, self.width = 6, 4

        # defining the states of our robot
        self.GO, self.IN_LENGTH, self.counter = 1, 1, 0
        self.steps, xp, yp = []
        self.X1, self.Y1, self.X2, self.Y2, self.X3, self.Y3, self.X4, self.Y4 = [], [], [], [], [], [], [], []

    
    def shape():
        self.X1 = np.linspace(0, 6 , 100)
        self.Y1 = np.array([4]*100)

        self.Y2 = np.linspace(4, 0, 100)
        self.X2 = np.array([6]*100)

        self.X3 = np.linspace(6, 0, 100)
        self.Y3 = np.array([0]*100)

        self.Y4 = np.linspace(0, 4, 100)
        self.X4 = np.array([0]*100)
  
    
    def move(self):
        traveled, steps = 0, 0
        if self.IN_LENGTH != 1:
                steps = self.width
                self.IN_LENGTH = 1
        else:
                steps = self.length
                self.IN_LENGTH = 0

        msg = rospy.wait_for_message("/odom" , Odometry) 
        start_x = msg.pose.pose.position.x
        start_y = msg.pose.pose.position.y
        while steps > traveled:
            twist.angular.z = 0
            twist = Twist()
            twist.linear.x = self.linear_speed
            self.cmd_publisher.publish(twist)
            msg = rospy.wait_for_message("/odom" , Odometry) 
            yp1= msg.pose.pose.position.y
            self.yp.append(msg.pose.pose.position.y)
            xp1= msg.pose.pose.position.x
            self.yp.append(msg.pose.pose.position.x)
    

    def run(self):
        self.shape()
        while not rospy.is_shutdown():
            self.move()
            self.cmd_publisher.publish(Twist())
            rospy.sleep(1)
            if self.counter == 3 :
                break
            self.counter += 1
            
            self.cmd_publisher.publish(Twist())
            rospy.sleep(1)
            
            msg = rospy.wait_for_message("/odom" , Odometry) 
            orientation = msg.pose.pose.orientation_, _, angle1 = 
            tf.transformations.euler_from_quaternion((orientation.x ,
            orientation.y ,orientation.z ,orientation.w))
            twist = Twist()

            twist.angular.z = self.angular_speed
            self.cmd_publisher.publish(twist)
            angle2 = angle1      

            while self.goal_angle + 1 > abs(angle1 - angle2):
                msg = rospy.wait_for_message("/odom" , Odometry)
                orientation = msg.pose.pose.orientation_, _, temp_angle 
                = tf.transformations.euler_from_quaternion(
                    (orientation.x ,orientation.y ,orientation.z ,orientation.w))
                if temp_angle > 0:
                    angle2 -= abs(abs(temp_angle) - abs(angle2))
                else :
                    angle2 = temp_angle
                    
            self.cmd_publisher.publish(Twist())
            rospy.sleep(1) 
            self.cmd_publisher.publish(Twist())
            rospy.sleep(1)
        self.Deviation_path()
              

    def Deviation_path(self):
        points = []
        zip_object1 = np.concatenate([X1, X2, X3, X4])
        zip_object2 = np.concatenate([Y1, Y2, Y3, Y4])
        for X2, Y2 in zip(self.xp, self.yp):
            minimum = 2000
            for X1, Y1 in zip(zip_object1, zip_object2):
                distance = dist([X2, Y2], [X1, Y1])
                if distance < minn:
                    minimum = distance
            points.append(minimum)
        plt.figure(1)
        plt.plot(points, color='b')
        plt.show()         

if __name__ == "__main__":
    controller = Controller()
    controller.run()