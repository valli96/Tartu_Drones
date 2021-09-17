#!/usr/bin/python3

import rospy
import math
from geometry_msgs.msg import Twist


def go_circle():
    # publisher start a new node
    rospy.init_node('turtle_circle', anonymous=True)
    velocity_publisher = rospy.Publisher(
        '/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # user imput
    print("Let's move:")
    radius_circual = float(input("Radius of the circle: "))
    speed = float(input("Imput speed: "))
    degree_goal = float(input("How many degrees: "))

    # inizial of speed (lin/rot)
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    # math for circule motion (checking for acuall pos)
    while not rospy.is_shutdown():

        t0 = rospy.Time.now().to_sec()
        actual_degree = 0

        while(actual_degree <= degree_goal):
            vel_msg.linear.x = math.sin(actual_degree)*speed
            vel_msg.linear.y = math.cos(actual_degree)*speed
            velocity_publisher.publish(vel_msg)

            t1 = rospy.Time.now().to_sec()
            catual_x = math.sin(actual_degree)*radius_circual*speed*(t1-t0)
            catual_y = math.cos(actual_degree)*radius_circual*speed*(t1-t0)
            actual_degree = speed*(t1-t0)*2*math.pi

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        go_circle()
    except rospy.ROSInterruptException:
        pass
