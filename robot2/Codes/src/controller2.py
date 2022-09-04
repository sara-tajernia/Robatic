#!/usr/bin/python3

from cmath import rect
from matplotlib import pyplot as plt
import numpy as np
import rospy
import tf
import math

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point

from math import atan2, pi, radians, sqrt

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)

        # getting specified parameters
        self.linear_speed = rospy.get_param("/controller/linear_speed") # m/s
        self.angular_speed = rospy.get_param("/controller/angular_speed") # rad/s
        self.goal_angle = radians(rospy.get_param("/controller/goal_angle")) # rad
        self.epsilon = rospy.get_param("/controller/epsilon")
        self.distanceThreshold = 0.3
        self.twist = Twist()
        
        
        # defining the states of our robot
        self.GO, self.ROTATE = 0, 1
        self.state = self.GO 

        self.kp_distance = 15
        self.ki_distance = 0.000001
        self.kd_distance = 0.4

        self.kp_angle = 3
        self.ki_angle = 0.03
        self.kd_angle = 0.05

        self.deviation_path = []
        

    # heading of the robot 
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        _, _, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        return yaw        
    def run(self):

        rectangle = []
        x1 = np.linspace(2, -2 , 20)
        rectangle = [*rectangle, *[[i, 3.0] for i in x1]]
        x2 = np.linspace(-2, 2 , 20)
        rectangle = [*rectangle, *[[i, -3.0] for i in x2]]
        y1 = np.linspace(3, -3 , 10)
        rectangle = [*rectangle, *[[2.0, i] for i in y1]]
        y2 = np.linspace(-3, 3 , 10)
        rectangle = [*rectangle, *[[-2.0, i] for i in y2]]
        
        self.path = rectangle
       
        while not rospy.is_shutdown():
            angle1 = 0
            for [x, y] in self.path:
                td, ta, dp_dis = 0, 0, 0
                self.cmd_publisher.publish(Twist())
                msg = rospy.wait_for_message("/odom" , Odometry) 
                x1 = msg.pose.pose.position.x
                y1 = msg.pose.pose.position.y
                xd = x - x1
                delta_y = y - y1
                dis = sqrt(xd**2 + delta_y**2)

                while dis > 0.1:
                    current_angle = self.get_heading()
                    msg = rospy.wait_for_message("/odom" , Odometry) 
                    x1 = msg.pose.pose.position.x
                    y1 = msg.pose.pose.position.y
                    self.deviation_path.append(sqrt(min([((x1 - i) ** 2 + (y1 - j) ** 2) for [i, j] in self.path])))
                    xd = x - x1
                    delta_y = y - y1
                    ap = atan2(delta_y , xd) 
                    if angle1 > pi-0.1 and current_angle <= 0:
                        current_angle = 2*pi + current_angle
                    elif angle1 < -pi+0.1 and current_angle > 0:
                        current_angle = -2*pi + current_angle
                        
                    if ap < -pi/4 or ap > pi/4:
                        if y < 0 and y1 < y:
                            ap = -2*pi + ap
                        elif y >= 0 and y1 > y:
                            ap = 2*pi + ap

                    dis = sqrt(xd**2 + dy**2)
                    diff_dis = dis - dp_dis
                    control_signal_dis = self.kp_dis*dis + self.ki_dis*td + self.kd_dis*diff_dis
                    control_signal_angle = self.kp_angle*(ap - current_angle)

                    self.twist.angular.z = (control_signal_angle)
                    self.twist.linear.x = min(control_signal_dis,0.1)
                    if self.twist.angular.z > 0:
                        self.twist.angular.z = min(self.twist.angular.z, 1.5)
                    else:
                        self.twist.angular.z = max(self.twist.angular.z, -1.5)
                    
                    angle1 = current_angle
                    self.cmd_publisher.publish(self.twist)
                    td += dis
                    dp_dis = dis
                    ta += ap
            self.cmd_publisher.publish(Twist())
            plt.plot(list(range(1, len(self.deviation_path) + 1)), self.deviation_path)
            plt.show()
            break

if __name__ == "__main__":
    controller = Controller()
    controller.run()