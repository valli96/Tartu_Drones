import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Twist, Point
import math


kpv = 1
kpav = 1.2

# pub velocity for turtlebot
rospy.init_node("speed_controller")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)


def imput_points():
    global goal
    goal = Point()
    goal.x = float(input("Input goal X position: "))
    goal.y = float(input("Input goal Y position: "))


def diff_angels(x, y):
    x += math.pi
    y += math.pi
    a = (x - y) % (math.pi*2)
    b = (y - x) % (math.pi*2)
    return -a if a < b else b


def dist(x, y):
    global goal
    inc_x = goal.x - x
    inc_y = goal.y - y
    distance = math.sqrt(inc_x**2 + inc_y**2)
    return inc_x, inc_y, distance


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

    inc_x, inc_y, distance = dist(x, y)
    # print(inc_x, "inc_x")
    # print(inc_y, "inc_y")
    angle_to_goal = math.atan2(inc_y, inc_x)
    print(angle_to_goal, "angle to the goal")
    print(theta, "theta")
    diff_ang = diff_angels(angle_to_goal, theta)
    print(diff_ang, "diff_ang")
    if distance > 0.2:
        if abs(angle_to_goal - theta) > 0.4:
            if (angle_to_goal - theta) > 0:
                speed.linear.x = 0.0
                speed.angular.z = 0.3*abs(angle_to_goal-theta) * \
                    kpav  # counter clockwise
                print("I move counter clockwise with: ", speed.angular.z)
            if (angle_to_goal - theta) < 0:
                speed.linear.x = 0.0
                speed.angular.z = -0.3 * \
                    abs(angle_to_goal-theta)*kpav  # clockwise
                print("I move clockwise", speed.angular.z)
        else:
            speed.linear.x = 0.5*distance*kpv
            speed.angular.z = 0.0
    # else:
    # speed.linear.x = 0.0
    # speed.angular.z = 0.1
    # print(theta)
    # print(speed.linear.x, "linear")
    # print(theta, "real angle")
    # print(angle_to_goal, "angle to the goal")
    # print(angle_to_goal-theta, "differnece")
    pub.publish(speed)


def get_goal_point(msg):
    global goal, theta
    # goal = Point()
    goal.x = msg.Point.x
    goal.y = msg.Point.y
    (roll, pitch, theta) = euler_from_quaternion(
        [rot_q.Quaternion.x, rot_q.Quaternion.y, rot_q.Quaternion.z, rot_q.Quaternion.w])


def main():

    # global goal, theta
    imput_points()
    # goal = Point()
    sub_inital = rospy.Subscriber(
        "/turtlebot/goal_pose", Pose, get_goal_point, queue_size=1)
    # imput_points()
    sub = rospy.Subscriber("/odom", Odometry, callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
