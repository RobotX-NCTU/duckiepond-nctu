#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, PoseArray, Point
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer
import message_filters
import math
import tf
fo = open("lidar.txt", "w")

def cb_pos(msg):
    robot_a = Point()
    robot_b = Point()
    robot_a = msg.poses[0].position
    robot_b = msg.poses[1].position
    dis = distance(robot_a, robot_b)
    #phi = angle(msg_follow.pose.pose.orientation, msg_lead.pose.pose.orientation)
    
    print(dis)

    text = str(dis) + '\n'
    fo.write(text)

def angle(orientation_1, orientation_2):
    q = (orientation_1.x, orientation_1.y, orientation_1.z, orientation_1.w)
    yaw_1 = tf.transformations.euler_from_quaternion(q)[2]

    q = (orientation_2.x, orientation_2.y, orientation_2.z, orientation_2.w)
    yaw_2 = tf.transformations.euler_from_quaternion(q)[2]

    yaw_dff = yaw_2 - yaw_1

    if yaw_dff >= math.pi:
        yaw_dff -= 2*math.pi
    elif yaw_dff <= -math.pi:
        yaw_dff += 2*math.pi

    return yaw_dff

def distance(position_1, position_2):
    x = position_1.x - position_2.x
    y = position_1.y - position_2.y
    return math.sqrt(x*x + y*y)

if __name__ == "__main__":
    rospy.init_node("odometry_plot",anonymous=False)

    point_list_follow = []
    point_list_lead = []

    # Square
    route_list = [(0, 7), (0, -21), (28, -21), (28, 7), (0, 7)]

    # Circle
    rospy.Subscriber("/pos", PoseArray, cb_pos, queue_size=1)

    rospy.spin()

fo.close()