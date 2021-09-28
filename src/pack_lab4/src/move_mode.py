import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

# x = 0.0
# y = 0.0
# theta = 0.0
rospy.init_node("speed_controller")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)


def imput_points():
    global goal
    goal = Point()
    goal.x = float(input("Input goal X position: "))
    goal.y = float(input("Input goal Y position: "))


def callback(msg):
    # global x, y, theta, goal
    global goal
    # goal.x = 8
    # goal.y = -3
    speed = Twist()
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    inc_x = goal.x - x
    inc_y = goal.y - y

    angle_to_goal = atan2(inc_y, inc_x)

    # print(theta, "the current angle")
    # print(angle_to_goal, "the angles the to points")

    if abs(angle_to_goal - theta) > 0.4:
        if (angle_to_goal - theta) > 0.4:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        if (angle_to_goal - theta) < 0.4:
            speed.linear.x = 0.0
            speed.angular.z = -0.3
    else:
        speed.linear.x = 0.5
        speed.angular.z = 0.0

    print(speed.linear.x, "linear")
    print(speed.angular.z, "angular")
    pub.publish(speed)


def main():
    imput_points()
    sub = rospy.Subscriber("/odom", Odometry, callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
