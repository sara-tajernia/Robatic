#!/usr/bin/python3

from cmath import rect
import rospy
from nav_msgs.msg import Odometry
from math import sqrt
from s1.msg import co


def Barriers(name, distance):
    barriers = calculate()
    barriers.obstacle_name = name
    barriers.distance = distance
    return barriers  

class distances1:
    
    def __init__(self) -> None: 
        self.default, self.book_shelf, self.dumpster, self.barrel, self.postbox, self.cabinet, self.cafe_table, self.fountain = 
        -1, (2.64, -1.55), (1.23, -4.57), (-2.51, -3.08), (-4.47, -0.57), (-3.44, 2.75), (-0.45, 4.05), (1.91, 3.37), (4.08, 1.14)
        self.obstacles = [("book_shelf", self.book_shelf), ("dumpster", self.dumpster), ("barrel", self.barrel), ("postbox", self.postbox), 
                            ("brick_box", self.brick_box), ("cabinet", self.cabinet), ("cafe_table", self.cafe_table), ("fountain", self.fountain)]
        rospy.init_node("controller" , anonymous=False)
        self.cal = rospy.Publisher('Barriers', calculate, queue_size=10)


    def calculate_distance(self):
        while True:
            if rospy.is_shutdown():
                return
            rate=rospy.Rate(1)
            msg = rospy.wait_for_message("odom" , Odometry) 
            curr_x, curr_y = msg.pose.pose.position.x, msg.pose.pose.position.y
            x_dis, y_dis = obstacle[1][0] - curr_x, obstacle[1][1] - curr_y
            distances = [(obstacle[0], sqrt((x_dis*x_dis) + (y_dis*y_dis))) for obstacle in self.obstacles]
            minimum = min(distances, key= lambda x: x[1])
            calculate = Barriers(minimum[0], minimum[1])
            self.cal.publish(calculate)
            rate.sleep()


if __name__ == "__main__":
    distance = distances1()
    distance.calculate_distance()

    