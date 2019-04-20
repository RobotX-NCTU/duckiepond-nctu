#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer
import message_filters
import math
import tf
from std_msgs.msg import Int32
counter = 0
def cb_odom(msg_folow, msg_lead):
    dis = distance(msg_folow.pose.pose.position, msg_lead.pose.pose.position)
    phi = angle(msg_folow.pose.pose.orientation, msg_lead.pose.pose.orientation)
    
    print(dis, phi)
    global counter
    counter += 1
    print("Counter = " + str(counter))

    text = str(dis) + " " + str(phi) + '\n'
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

def cb_clear(msg_int):
    print("Counter Clear point = ", counter)
    print("--------------------------------------")

if __name__ == "__main__":
    rospy.init_node("odometry_plot",anonymous=False)

    point_list_follow = []
    point_list_lead = []

    file_ = rospy.get_param("~file")
    fo = open(file_+".txt", "w")


    rospy.Subscriber("/clear", Int32, cb_clear, queue_size=1)

    sub_odom_follow = message_filters.Subscriber("/MONICA/localization_gps_imu/odometry", Odometry)
    sub_odom_lead = message_filters.Subscriber("/BRIAN/localization_gps_imu/odometry", Odometry)

    ats = ApproximateTimeSynchronizer((sub_odom_follow, sub_odom_lead), queue_size = 1, slop = 0.5)
    ats.registerCallback(cb_odom)

    rospy.spin()

fo.close()