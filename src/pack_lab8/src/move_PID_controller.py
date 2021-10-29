#!/usr/bin/env python3

from geometry_msgs.msg import Pose, Twist, Point
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import math
import time
import rospy
# import PID_controller_class


class PID_controller:
    def __init__(self, kp_v_x,  kp_v_y, kp_v_z, kp_av, ki_v_x, ki_v_y, ki_v_z, ki_av, kd_v_x, kd_v_y, kd_v_z, kd_av, ki_v_x_min, ki_v_y_min, ki_v_z_min, ki_av_min, ki_v_x_max, ki_v_y_max, ki_v_z_max, ki_av_max):

        self._kp_v_x = kp_v_x
        self._kp_v_y = kp_v_y
        self._kp_v_z = kp_v_z
        self._kp_av = kp_av

        self._ki_v_x = ki_v_x
        self._ki_v_y = ki_v_y
        self._ki_v_z = ki_v_z
        self._ki_av = ki_av

        self._kd_v_x = kd_v_x
        self._kd_v_y = kd_v_y
        self._kd_v_z = kd_v_z
        self._kd_av = kd_av

        self._ki_v_x_min = ki_v_x_min
        self._ki_v_y_min = ki_v_y_min
        self._ki_v_z_min = ki_v_z_min
        self._ki_av_min = ki_av_min

        self._ki_v_x_max = ki_v_x_max
        self._ki_v_y_max = ki_v_y_max
        self._ki_v_z_max = ki_v_z_max
        self._ki_av_max = ki_av_max

        self.dt = None
        self._last_time = None
        self._linear_error_last_x = None
        self._linear_error_last_y = None
        self._linear_error_last_z = None
        self._angular_error_last = None

    def calcualte_time_loop(self):
        """
        start loop timing and calculate the time of one loop
        first_loop -> dt = 0
        """
        actual_time = time.time()

        if self._last_time == None:
            self._last_time = actual_time

        self.dt = actual_time - self._last_time
        self._last_time = actual_time

    def update_PID_contoller(self, linear_error_x, linear_error_y, linear_error_z, angular_error):
        """
        linear_error:= x and y difference between actual position and goal position
        angular_error:= angular difference between actual angel and the angel to the goal

        The actual implementation  of the PID controller.

        """

        self.calcualte_time_loop()
        if self.dt == 0:
            self._linear_controler_x = 0
            self._linear_controler_y = 0
            self._linear_controler_z = 0
            self._angular_controler = 0
            return self._linear_controler, self._angular_controler,

        # P controller part
        linear_velocity_p_x = self._kp_v_x * linear_error_x
        linear_velocity_p_y = self._kp_v_y * linear_error_y
        linear_velocity_p_z = self._kp_v_z * linear_error_z
        angular_velocity_p = self._kp_av * angular_error

        if(self._linear_error_x_last == None):
            self._linear_error_x_last = linear_error_x
        if(self._linear_error_y_last == None):
            self._linear_error_y_last = linear_error_y
        if(self._linear_error_z_last == None):
            self._linear_error_z_last = linear_error_z

        if(self._angular_error_last == None):
            self._angular_error_last = angular_error

        # D controller part
        linear_velocity_d_x = self._kd_v_x * \
            (self._linear_error_x_last - linear_error_x)/self.dt
        linear_velocity_d_y = self._kd_v_y * \
            (self._linear_error_y_last - linear_error_y)/self.dt
        linear_velocity_d_z = self._kd_v_z * \
            (self._linear_error_z_last - linear_error_z)/self.dt

        angular_velocity_d = self._kd_av * \
            (self._angular_error_last - angular_error)/self.dt

        # linear_velocity_i += self.dt*self._kp_v

        self._linear_error_x_last = linear_error_x
        self._linear_error_y_last = linear_error_y
        self._linear_error_z_last = linear_error_z
        self._angular_error_last = angular_error

        print(linear_velocity_p_x, "linear_velocity_p")
        print(linear_velocity_d_x, "linear_velocity_d")
        print(linear_velocity_p_y, "linear_velocity_p")
        print(linear_velocity_d_y, "linear_velocity_d")
        print(linear_velocity_p_z, "linear_velocity_p")
        print(linear_velocity_d_z, "linear_velocity_d")

        print(angular_velocity_p, "angular_velocity_p")
        print(angular_velocity_d, "angular_velocity_d")

        self._linear_controler_x = linear_velocity_p_x + linear_velocity_d_x
        self._linear_controler_y = linear_velocity_p_y + linear_velocity_d_y
        self._linear_controler_z = linear_velocity_p_z + linear_velocity_d_z
        self._angular_controler = angular_velocity_p + angular_velocity_d

        # limits speed
        if abs(self._linear_controler_x) >= 0.5:  # should not be negative
            self._linear_controler_x = 0.5
        if abs(self._linear_controler_y) >= 0.5:  # should not be negative
            self._linear_controler_y = 0.5
        if abs(self._linear_controler_z) >= 0.5:  # should not be negative
            self._linear_controler_z = 0.5


# tuning parameters
kp_v_x = 0.2
kp_v_y = 0.2
kp_v_z = 0.2
kp_av = 0.7

ki_v_x = 0.5
ki_v_y = 0.5
ki_v_z = 0.5
ki_av = 0.02

kd_v_x = 0.5
kd_v_y = 0.5
kd_v_z = 0.5
kd_av = 0.2

ki_v_x_min = 0.1
ki_v_y_min = 0.1
ki_v_z_min = 0.1
ki_av_min = 0.1

ki_v_x_max = 0.7
ki_v_y_max = 0.7
ki_v_z_max = 0.7
ki_av_max = 0.7

# pub velocity for turtlebot
rospy.init_node("speed_controller")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)


PID_class = PID_controller(
    kp_v_x, kp_v_z, kp_v_z, kp_av, ki_v_x, ki_v_y, ki_v_z, ki_av, kd_v_x, kd_v_y, kd_v_z,  kd_av)


def imput_points():
    global goal
    goal = Point()
    goal.x = float(input("Input goal X position: "))
    goal.y = float(input("Input goal Y position: "))
    goal.z = float(input("Input goal Z position: "))


def diff_angels(x, y):
    x += math.pi
    y += math.pi
    a = (x - y) % (math.pi*2)
    b = (y - x) % (math.pi*2)
    return -a if a < b else b


def dist(x, y, z):
    global goal
    inc_x = goal.x - x
    inc_y = goal.y - y
    inc_z = goal.z - z
    distance = math.sqrt(inc_x**2 + inc_y**2 + inc_z**2)
    return inc_x, inc_y, inc_z, distance


def get_goal_point(msg):
    global goal, theta
    goal.x = msg.Point.x
    goal.y = msg.Point.y
    goal.z = msg.Point.z
    (roll, pitch, theta) = euler_from_quaternion(
        [rot_q.Quaternion.x, rot_q.Quaternion.y, rot_q.Quaternion.z, rot_q.Quaternion.w])


def callback(msg):
    global goal
    speed = Twist()
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    inc_x, inc_y, inc_z, distance_to_goal = dist(x, y, z)
    angle_to_goal = math.atan2(inc_y, inc_x, inc_z)
    diff_ang = -diff_angels(angle_to_goal, theta)

    if distance_to_goal > 0.2:
        PID_class.update_PID_contoller(
            distance_to_goal, diff_ang)

    speed.linear.x = PID_class._linear_controler_x
    speed.linear.y = PID_class._linear_controler_y
    speed.linear.z = PID_class._linear_controler_z
    speed.angular.z = PID_class._angular_controler
    # speed.angular.z = 0

    pub.publish(speed)


def main():
    imput_points()

    sub_inital = rospy.Subscriber(
        "/drone/goal_pose", Pose, get_goal_point, queue_size=1)
    sub = rospy.Subscriber("/odom", Odometry, callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
