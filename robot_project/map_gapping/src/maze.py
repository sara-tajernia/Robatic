#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

class PIDController():


    def __init__(self):
        
        rospy.init_node('follow_maze', anonymous=False)
        
        self.k_p = 3
        self.k_i = 0
        self.k_d = 22
        
        self.dt = 0.005
        self.v = 0.2
        self.D = 0.5
        rate = 1/self.dt
        
        self.r = rospy.Rate(rate)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)


    
    def follow_wall(self):
                
        sum_i_theta = 0
        prev_theta_error = 0
        
        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = self.v


        while not rospy.is_shutdown():
            laser_data = rospy.wait_for_message("/scan" , LaserScan)
            front_d = min(laser_data.ranges[0:5])
            d = min(laser_data.ranges[5:180])  

            if front_d <= self.D :
                twist = Twist()
                twist.angular.z = -0.2
                twist.linear.x = 0.0
                self.cmd_vel.publish(twist)
            
            else:
                self.cmd_vel.publish(move_cmd)

                err = d - self.D
                sum_i_theta += err * self.dt
                
                P = self.k_p * err
                I = self.k_i * sum_i_theta
                D = self.k_d * (err - prev_theta_error)

                move_cmd.angular.z = P + I + D 
                prev_theta_error = err
                move_cmd.linear.x = self.v            
                                                

                self.r.sleep()

            

if __name__ == '__main__':
    pidc = PIDController()
    pidc.follow_wall()

