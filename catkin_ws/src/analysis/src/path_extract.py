#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer
import message_filters
import math
import tf
fo = open("combine_lidar_square.txt", "w")
counter = 0
def cb_odom(msg_follow, msg_lead):
    global counter
    if len(msg_follow.poses) is 0 or  len(msg_lead.poses) is 0:
        return
    dis = distance(msg_follow.poses[len(msg_follow.poses)-1].pose.position, msg_lead.poses[len(msg_lead.poses)-1].pose.position)
    
    print(dis, counter)
    if dis >= 10:
        return
    counter += 1

    text = str(dis) + '\n'
    fo.write(text)

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
    sub_odom_follow = message_filters.Subscriber("/path_a", Path)
    sub_odom_lead = message_filters.Subscriber("/path_b", Path)

    ats = ApproximateTimeSynchronizer((sub_odom_follow, sub_odom_lead), queue_size = 1, slop = 0.5)
    ats.registerCallback(cb_odom)

    rospy.spin()

fo.close()
