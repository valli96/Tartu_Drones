#!/usr/bin/env python3

from geometry_msgs.msg import Pose, Twist, Point
from ar_track_alvar_msgs.msg import AlvarMarkers
import math
import time
import rospy

# from tf.transformations import euler_from_quaternion
# from nav_msgs.msg import Odometry
# from std_msgs.msg import Empty


rospy.init_node("pose_detection")


def callback(msg):
    print(msg.pose.pose.position)
    print(msg.pose.pose.orientation)


def myhook():
    print("shutdown time!")


rospy.on_shutdown(myhook)


def main():
    time.sleep(1)
    sub = rospy.Subscriber(
        "/ar_track_alvar", AlvarMarkers, callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    while not rospy.is_shutdown():
        main()
