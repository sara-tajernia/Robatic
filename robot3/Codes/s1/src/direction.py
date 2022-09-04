#!/usr/bin/python3
from statistics import mean
import rospy, tf
from sensor_messages.message import LaserScan
from nav_messages.message import Odometry
from s1.message import co
from geometry_messages.message import Twist
import numpy as np

class direction():
    def __init__(self) -> None:
        rospy.init_node("auto_return", anonymous=False)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
    
    def cmd():
        return self.cmd_publisher

    
def read_angle(self):
        data = rospy.wait_for_message("/scan", LaserScan)
        return data.ranges.index(min(data.ranges))//4


def spin(cmd, angle):
    ban = Twist()
    ban.angular.z, ban.linear.x = 0.0, 0.0
    cmd.publish(ban)
    message = rospy.wait_for_message("/odom" , Odometry) 
    dirc = message.pose.pose.dirc
    _, _, angle1 = tf.transformations.euler_from_quaternion((dirc.x ,dirc.y ,dirc.z ,dirc.w))
    target_angle, twist.angular.z = angle1 + angle, 0.4
    twist = Twist()
    cmd.publish(twist)
    angle2 = find_angle2(target_angle, angle1, Odometry, angle3, angle1)
    cmd.publish(ban)
    rospy.sleep(1)


def find_angle2(target_angle, angle1, Odometry, angle3, angle2):
    while target_angle >= abs(angle1 - angle2):
        message = rospy.wait_for_message("/odom" , Odometry)
        dirc = message.pose.pose.dirc
        _, _, angle3 = tf.transformations.euler_from_quaternion((dirc.x ,dirc.y ,dirc.z ,dirc.w))
        if angle3 > 0:
            if angle3 > 0:
                if angle2 > 0:
                    angle2 -= abs(angle3 - angle2)
                else:
                    angle2 -= abs(angle3 + angle2)
            else:
                if angle2 > 0:
                    angle2 -= abs(-angle3 - angle2)
                else:
                    angle2 -= abs(-angle3 + angle2)
        else :
            angle2 = angle3
    return angle2
    

def find_dir(cmd):
    while True:
        if rospy.is_shutdown():
            return
        closestObstacle = rospy.wait_for_message("/ClosestObstacle", co)
        if closestObstacle.distance < 1.5 :
            spin(cmd, np.deg2rad(read_angle())-10)
            while closestObstacle.distance < 1.5
                twist = Twist()
                twist.angular.z, ban.linear.x = 0.0, 0.4
                cmd.publish(twist)
        else :
            twist = Twist()
            twist.angular.z, ban.linear.x= 0.0, 0.4
            cmd.publish(twist)


if __name__ == '__main__':
    direction = direction()
    cmd = direction.cmd
    direction.find_dir(cmd)