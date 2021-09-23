#!/usr/bin/python3

import rospy
import math
# from nav_msgs.msg import Odometry  # for the position
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def rotate_90():
    epoch = rospy.Time()
    degree = 0
    print("I am in rotaion")
    while degree <= 90:
        degree = vel.angular.z*(rospy.get_time() - epoch)
    obstacle = False


class rotaion(object):
    def __init__(self, pub):
        self._obstacle = False
        self._time_roataion = 3
        self._vel = Twist()

    def callback(self, msg):
        print(dt.ranges[0])

        rospy.loginfo(LaserScan.ranges)
        if msg.ranges[0] < 0.2:
            print("to close")
            obstacle = True
            rotate_90()


def main():
    # node definition
    rospy.init_node('node_move_90deg')
    # define publisher and subscriber
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    Rotator = rotaion(pub)
    rospy.Subscriber('/scan', LaserScan, rotaion.callback)

    # keeps python form exiting
    rospy.spin()


if __name__ == '__main__':
    main()
