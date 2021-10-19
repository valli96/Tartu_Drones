import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Twist, Point
import math

# tuning parameters
kpv = 0.5
kpav = 1

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
    speed = Twist()
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    inc_x, inc_y, distance = dist(x, y)
    angle_to_goal = math.atan2(inc_y, inc_x)
    diff_ang = diff_angels(angle_to_goal, theta)
    # print(angle_to_goal, "angle to the goal")
    # print(theta, "theta")
    # print(diff_ang, "diff_ang")
    # print(angle_to_goal-theta, "angle_to_goal-theta")
    if distance > 0.2:
        if abs(-diff_ang) > 0.4:
            if (diff_ang) > 0:
                speed.linear.x = 0.0
                speed.angular.z = -0.3*abs(diff_ang) * \
                    kpav  # counter clockwise
                print("I move counter clockwise with: ", speed.angular.z)
            if (diff_ang) < 0:
                speed.linear.x = 0
                speed.angular.z = 0.3 * \
                    abs(diff_ang)*kpav  # clockwise
                print("I move clockwise", speed.angular.z)
        else:
            speed.linear.x = 0.5*distance*kpv
            speed.angular.z = 0.0
        print(diff_ang)

    # speed.linear.x = 0
    pub.publish(speed)


def get_goal_point(msg):
    global goal, theta
    # goal = Point()
    goal.x = msg.Point.x
    goal.y = msg.Point.y
    (roll, pitch, theta) = euler_from_quaternion(
        [rot_q.Quaternion.x, rot_q.Quaternion.y, rot_q.Quaternion.z, rot_q.Quaternion.w])


def main():

    imput_points()
    sub_inital = rospy.Subscriber(
        "/turtlebot/goal_pose", Pose, get_goal_point, queue_size=1)
    sub = rospy.Subscriber("/odom", Odometry, callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
