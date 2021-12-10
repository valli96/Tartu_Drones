#!/usr/bin/env python3

from geometry_msgs.msg import Pose, Twist, Point
# from ar_track_alvar_msgs.msg import AlvarMarkers
from visualization_msgs.msg import Marker
import math
import time
import rospy

# from tf.transformations import euler_from_quaternion
# from nav_msgs.msg import Odometry
# from std_msgs.msg import Empty


rospy.init_node("pose_detection")


def callback(msg):
    print(msg)
    print("position")
    print(msg.pose.position.x, "this ist x")
    print("orientation")
    print(msg.pose.orientation)
    # print("position")
    # print(msg.pose.pose.position)
    # print("orientation")
    # print(msg.pose.pose.orientation)


def myhook():
    print("shutdown time!")


rospy.on_shutdown(myhook)


def main():
    time.sleep(1)
    print("bevor_callback")
    sub = rospy.Subscriber("/visualization_marker",
                           Marker, callback, queue_size=1)
    # sub = rospy.Subscriber("/ar_pose_marker",
    #                        AlvarMarkers, callback, queue_size=1)
    print("after_callback")
    rospy.spin()


if __name__ == '__main__':
    while not rospy.is_shutdown():
        main()
