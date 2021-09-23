#! /usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
# from sensor_msgs.msg import LaserScan  # laser date


def callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    # rospy.loginfo('x:{}, y:{}'.format(x, y))
    print("Halle")


def main():
    rospy.init_node("get_Pos_node")
    rospy.Subscriber("/odom", Odometry, callback)
    rospy.spin()


if __name__ == "__main__":
    main()
