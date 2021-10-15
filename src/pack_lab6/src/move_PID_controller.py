#!/usr/bin/env python3

from geometry_msgs.msg import Pose, Twist, Point
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import math
import time
import rospy
# import PID_controller_class


class PID_controller:
    def __init__(self, kp_v, kp_av, ki_v, ki_av, kd_v, kd_av):

        self._kp_v = kp_v
        self._kp_av = kp_av
        self._ki_v = ki_v
        self._ki_av = ki_av
        self._kd_v = kd_v
        self._kd_av = kd_av

        self._ki_v_min = ki_v
        self._ki_av_min = ki_av
        self._ki_v_max = ki_v
        self._ki_av_max = ki_av

        self._last_time = None
        self.dt = None
        self._linear_error_last = None
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

    def update_PID_contoller(self, linear_error, angular_error):
        """
        linear_error:= x and y difference between actual position and goal position
        angular_error:= angular difference between actual angel and the angel to the goal

        The actual implementation  of the PID controller.

        """
        print("linear_error",linear_error)
        print("angular_error",angular_error)

        self.calcualte_time_loop()
        if self.dt == 0:
            self._angular_controler = 0
            self._linear_controler = 0
            return self._angular_controler, self._linear_controler

        # P controller part
        linear_velocity_p = self._kp_v * linear_error
        angular_velocity_p = self._kp_av * angular_error

        if(self._linear_error_last == None):
            self._linear_error_last = linear_error
        if(self._angular_error_last == None):
            self._angular_error_last = angular_error

        # D controller part
        linear_velocity_d = self._kd_v * \
            (self._linear_error_last - linear_error)/self.dt
        angular_velocity_d = self._kd_av * \
            (self._linear_error_last - linear_error)/self.dt

        # linear_velocity_i += self.dt*self._kp_v

        self._linear_error_last = linear_error
        self._angular_error_last = angular_error

        # self._linear_controler = linear_velocity_p + linear_velocity_d
        # self._angular_controler = angular_velocity_p + angular_velocity_d
        
        self._linear_controler = abs(linear_velocity_p)
        self._angular_controler = angular_velocity_p

        # limits speed
        if self._linear_controler >= 0.5:
            self._linear_controler = 0.5
        
        # if abs(self._angular_controler) >= 0.5:
        #     if self._angular_controler >= 0.5:
        #         self._angular_controler = 0.5
        #     if self._angular_controler >= -0.5:
        #         self._angular_controler = -0.5
     
         
        
        print("published speed",self._linear_controler )
        print("published angular speed",self._angular_controler )



# tuning parameters
kp_v = 0.5
kp_av = 0.2

ki_v = 0.5
ki_av = 0.02

kd_v = 0.5
kd_av = 0.002

# pub velocity for turtlebot
rospy.init_node("speed_controller")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)


PID_class = PID_controller(
    kp_v, kp_av, ki_v, ki_av, kd_v, kd_av)


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


def get_goal_point(msg):
    global goal, theta
    goal.x = msg.Point.x
    goal.y = msg.Point.y
    (roll, pitch, theta) = euler_from_quaternion(
        [rot_q.Quaternion.x, rot_q.Quaternion.y, rot_q.Quaternion.z, rot_q.Quaternion.w])


def callback(msg):
    global goal
    speed = Twist()
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    inc_x, inc_y, distance_to_goal = dist(x, y)
    angle_to_goal = math.atan2(inc_y, inc_x)
    diff_ang = diff_angels(angle_to_goal, theta)

    if distance_to_goal > 0.2:
        PID_class.update_PID_contoller(
            distance_to_goal, diff_ang)
    
    # speed.linear.x = PID_class._linear_controler
    speed.angular.z = PID_class._angular_controler
    # speed.angular.z = 0
    print("distance to goal", distance_to_goal)
    print("angle to goal", diff_ang)
    pub.publish(speed)
 

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
