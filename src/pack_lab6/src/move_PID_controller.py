from geometry_msgs.msg import Pose, Twist, Point
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import math
import time
import rospy
import PID_controller_class

# import sys
# sys.path.append(".")


# tuning parameters
kp_v = 0.5
kp_av = 1

ki_v = 0.5
ki_av = 1

kd_v = 0.5
kd_av = 1


controller = PID_controller_class.PID_controller(
    kp_v, kp_av, ki_v, ki_av, kd_v, kd_av)
# pub velocity for turtlebot
rospy.init_node("speed_controller")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)


# def PID_controller(distance_to_goal, diff_ang):
#     # time
#     cur_time = time.time()
#     time_diff = cur_time-last_time

#     # proportional part
#     pro_vel = distance_to_goal*kp_v
#     pro_velang = abs(diff_angels)*kp_av
#     # integrative part
#     int_vel = (int_vel - distance_to_goal)*time_diff*ki_v
#     int_velang = (int_velang - abs(diff_angels))*time_diff*ki_av
#     # differential part
#     dif_vel = (dif_vel - distance_to_goal)/time_diff*kd_v
#     dif_velang = (dif_velanf - abs(diff_ang))/time_diff*kd_av

#     last_time = cur_time
#     return pro_vel, pro_velang, int_vel, int_velang, dif_vel, dif_velang


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
        controller = update_PID_contoller(distance_to_goal, diff_ang)
        # if abs(-diff_ang) > 0.4:
        #     if (-diff_ang) > 0:
        #         speed.linear.x = 0.0
        #         speed.angular.z = 0.3*abs(diff_ang) * \
        #             kpav  # counter clockwise
        #         print("I move counter clockwise with: ", speed.angular.z)
        #     if (-diff_ang) < 0:
        #         speed.linear.x = 0.0
        #         speed.angular.z = -0.3 * \
        #             abs(diff_ang)*kpav  # clockwise
        #         print("I move clockwise", speed.angular.z)
        # else:
        #     speed.linear.x = 0.5*distance_to_goal*kpv
        #     speed.angular.z = 0.0

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
