#!/usr/bin/env python3

from geometry_msgs.msg import Pose, Twist, Point
from std_msgs.msg import Empty
# from tf.transformations import euler_from_quaternion
# from nav_msgs.msg import Odometry
import math
import time
import rospy
# import PID_controller_class


# pub velocity for turtlebot
# pub_twist = rospy.Publisher("cmd_vel", Twist, queue_size=1)


# def callback(msg):
#     speed = Twist()
#     x = msg.pose.pose.position.x
#     y = msg.pose.pose.position.y
#     rot_q = msg.pose.pose.orientation
#     speed.linear.x = 1
#     speed.angular.z = 2
#     pub.publish(speed)

rospy.init_node("drone_control")
pub_takeoff = rospy.Publisher("drone/takeoff", Empty, queue_size=1)
pub_move = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
pub_land = rospy.Publisher("/drone/land", Empty, queue_size=1)
hey = Empty()
movment = Twist()


def main():
    time.sleep(0.5)
    while True:
        action = input(
            "What should the drone do?\n 1= take off \n 2= land\n 3= move\n")
        if action == "1":
            pub_takeoff.publish(hey)
        if action == "2":
            pub_land.publish(hey)
        if action == "3":
            print("Which direction")
            while True:
                move_dir = input()
                if move_dir == "h":
                    movment.linear.x = -1
                    movment.linear.y = 0
                    print("sidewayse")
                if move_dir == "j":
                    movment.linear.x = 0
                    movment.linear.y = -1
                    print("forwards")
                if move_dir == "k":
                    movment.linear.x = 0
                    movment.linear.y = 1
                    print("backwards")
                if move_dir == "l":
                    movment.linear.x = 1
                    movment.linear.y = 0
                    print("sidewayse")
                if move_dir == "0":
                    movment.linear.x = 0
                    movment.linear.y = 0
                pub_move.publish(movment)
                if move_dir == "q":
                    movment.linear.x = 0
                    movment.linear.y = 0
                    pub_move.publish(movment)
                    break
        action = "0"

    time.sleep(4)
    pub_land.publish(hey)

    # sub_inital = rospy.Subscriber(
    #     "/turtlebot/goal_pose", Pose, get_goal_point, queue_size=1)
    # sub = rospy.Subscriber("/odom", Odometry, callback, queue_size=1)
    # rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
