
#!/usr/bin/python3
import rospy
import math
# from nav_msgs.msg import Odometry  # for the position
# from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point


x = 0.0
y = 0.0
theta = 0.0


def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])


pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
move = Twist()


def movement_worng(x_curr, y_curr):
    # move in x
    if abs(goal_pos_x - x_curr) >= 0.05:
        # print("first")
        if (goal_pos_x - x_curr) > 0:
            move.linear.x = 0.5
        if (goal_pos_x - x_curr) < 0:
            move.linear.x = -0.5
    else:
        move.linear.x = 0
    # move in y
    if abs(goal_pos_y - y_curr) >= 0.05:
        # print("second")
        if (goal_pos_y - y_curr) > 0:
            move.linear.y = 0.5
        if (goal_pos_y - y_curr) < 0:
            move.linear.y = -0.5
    else:
        move.linear.y = 0

    # move.linear.x = 1
    # move.linear.y = 1
    print(move.linear.x, "movement in x")
    print(move.linear.y, "movement in y")
    pub.publish(move)


def callback(msg):

    # goal_pos_x = 0
    # goal_pos_y = 0
    x_curr = msg.pose.pose.position.x
    y_curr = msg.pose.pose.position.y
    print(x_curr, "current x pos")
    print(y_curr, "current y pos")
    # print(abs(goal_pos_x - x_curr), "x")
    # print(abs(goal_pos_y - y_curr), "y")
    # print(goal_pos_x, "x")
    # print(goal_pos_y, "y")

    movement_worng(x_curr, y_curr)


def main():

    global goal_pos_x
    global goal_pos_y
    goal_pos_x = float(input("input goal x position: "))
    goal_pos_y = float(input("input goal y position: "))
    rospy.init_node('move_to_pos', anonymous=True)
    # global pub
    # global move
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # move = Twist()
    # print(goal_pos_x, "x")
    # print(goal_pos_y, "y")
    sub = rospy.Subscriber('/odom', Odometry, callback)

    # keeps python form exiting
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
