import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Twist, Pose
from math import atan2


def main():
    rospy.init_node("position_pub")
    pub = rospy.Publisher("/turtlebot/goal_pose", Pose, queue_size=1)
    pose = Pose()
    pose.Point.x = 4
    pose.Point.y = 3
    roll = 0
    pitch = 0
    theta = math.pi/2
    [x, y, z, w] = quaternion_from_euler(
        [roll, pitch, theta])
    pose.Quaternion.x = x
    pose.Quaternion.y = y
    pose.Quaternion.z = z
    pose.Quaternion.w = w

    # pose.Quaternion.w = 1

    pub.publish(pose)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass
