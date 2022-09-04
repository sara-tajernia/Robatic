#!/usr/bin/python3

from cmath import rect
import rospy
from nav_messages.message import Odometry
from math import sqrt
from s1.srv import GetDistance, distance1

def find_distance():
    rospy.init_node('distance_node', anonymous=True)
    distance2 = distance()
    distance3 = rospy.Service('get_distance', GetDistance, get_distance)
    rospy.spin()

class Distance():
    def __init__(self) -> None:
        self.default, self.book_shelf, self.dumpster, self.barrel, self.postbox, self.cabinet, self.cafe_table, self.fountain = 
        -1, (2.64, -1.55), (1.23, -4.57), (-2.51, -3.08), (-4.47, -0.57), (-3.44, 2.75), (-0.45, 4.05), (1.91, 3.37), (4.08, 1.14)
        self.obstacles = {
            "book_shelf": self.book_shelf,
            "dumpster": self.dumpster,
            "barrel": self.barrel,
            "postbox": self.postbox,
            "brick_box": self.brick_box,
            "cabinet": self.cabinet,
            "cafe_table": self.cafe_table,
            "fountain": self.fountain
        }

    def get_distance(self, req):
        message = rospy.wait_for_message("odom" , Odometry)
        x_pos = message.pose.pose.position.x
        y_pos = message.pose.pose.position.y
        x_dis = self.obstacles[req.obstacle_name][0] - x_pos
        y_dis = self.obstacles[req.obstacle_name][1] - y_pos
        Distance = sqrt((x_dis*x_dis) + (y_dis*y_dis)
        distance1 = distance1()
        distance1.distance = distance
        return distance1      
        
    
if __name__ == '__main__':
    find_distance()

    