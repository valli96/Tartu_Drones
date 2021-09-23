#!/usr/bin/python3

# copied form enansakib 
# https://github.com/enansakib/obstacle-avoidance-turtlebot/blob/master/src/naive_obs_avoid_tb3.py

import rospy  # Python library for ROS
# LaserScan type message is defined in sensor_msgs
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def callback(dt):
    print('-------------------------------------------')
    print('Range data at 0 deg:   {}'.format(dt.ranges[0]))
    print('Range data at 15 deg:  {}'.format(dt.ranges[15]))
    print('Range data at 345 deg: {}'.format(dt.ranges[345]))
    print('-------------------------------------------')
    thr1=0.8  # Laser scan range threshold
    thr2=0.8
    # Checks if there are obstacles in front and
    if dt.ranges[0] > thr1 and dt.ranges[15] > thr2 and dt.ranges[345] > thr2:
                                                                         # 15 degrees left and right (Try changing the
									 # the angle values as well as the thresholds)
        move.linear.x=0.5  # go forward (linear velocity)
        move.angular.z=0.0  # do not rotate (angular velocity)
    else:
        move.linear.x=0.0  # stop
        move.angular.z=0.5  # rotate counter-clockwise
        if dt.ranges[0] > thr1 and dt.ranges[15] > thr2 and dt.ranges[345] > thr2:
            move.linear.x=0.5
            move.angular.z=0.0
    pub.publish(move)  # publish the move object


move=Twist()  # Creates a Twist message type object
rospy.init_node('obstacle_avoidance_node')  # Initializes a node
# Publisher object which will publish "Twist" type messages
pub=rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
                            				 # on the "/cmd_vel" Topic, "queue_size" is the size of the
                                                         # outgoing message queue used for asynchronous publishing

# Subscriber object which will listen "LaserScan" type messages
sub=rospy.Subscriber("/scan", LaserScan, callback)
                                                      # from the "/scan" Topic and call the "callback" function
						      # each time it reads something from the Topic

rospy.spin()  # Loops infinitely until someone stops the program execution
