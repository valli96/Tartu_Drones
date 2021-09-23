#!/usr/bin/python3
import rospy
import math
# from nav_msgs.msg import Odometry  # for the position
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# global variable
rotaion_time = 5
operation_mode = 0  # 0: no obstacle go forward   1: obstacle 90 degree turn

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
move = Twist()


def detect_obstacle(Laser_sub):
    move.linear.x = 0.3
    move.angular.z = 0
    dist = Laser_sub.ranges[0]
    print(dist)
    if dist < 0.45:
        print("obstacle in front of turtlebot")
        # rotate_90_new()

        global operation_mode
        operation_mode = 1


def rotate_90_new():
    move.linear.x = 0
    move.angular.z = (2*math.pi/4)/rotaion_time
    pub.publish(move)
    print("start_rotaion")
    rospy.sleep(rotaion_time)
    global operation_mode
    operation_mode = 0

    # move.linear.x = 0.2
    # move.angular.z = 0
    # pub.publish(move)


def move_forward():
    move.linear.x = 0.2
    move.angular.z = 0
    pub.publish(move)


def callback(Laser_sub):
    # t0 = rospy.Time.now().nsecs
    # move.linear.x = 0.5
    # move.angular.z = 0
    pub.publish(move)
    detect_obstacle(Laser_sub)
    global operation_mode
    if operation_mode == 1:
        rotate_90_new()
    if operation_mode == 0:
        move_forward()

    print(move.linear.x)


def main():
    rospy.init_node('node_move_90deg', anonymous=True)
    sub = rospy.Subscriber('/scan', LaserScan, callback, queue_size=1)
    # keeps python form exiting
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
