#!/usr/bin/python3
import rospy
import math
from geometry_msgs.msg import Twist


def move_circular():
    # Starts a new node
    rospy.init_node('turtlesim_move_circular_path', anonymous=True)
    # def publisher
    velocity_publisher = rospy.Publisher(
        '/turtle1/cmd_vel', Twist, queue_size=10)
    # use Twist masage
    vel_msg = Twist()

    # Receiveing the user's input
    print("The cript circual_turtlesim.py is running: ")
    radius = float(input("Type your radius: "))
    distance = radius*2*math.pi

    while not rospy.is_shutdown():

        vel_msg.linear.x = 0.3
        vel_msg.angular.z = 0.1*math.pi

        # Publish the velocity
        velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        # Testing our function
        move_circular()
    except rospy.ROSInterruptException:
        pass
