#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry


def callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rospy.loginfo('x:{}, y:{}'.format(x, y))


def main():
    rospy.init_node('node_move_90deg', anonymous=True)
    rospy.Subscriber("sub_Odometry", Odometry, callback)
    rospy.spin()


if __name__ == '__main__':
    main()
