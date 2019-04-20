#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from std_msgs.msg import Int32

point_list_follow = []
point_list_lead = []

def cb_odom_follow(msg):
    position = msg.pose.pose.position
    point_list_follow.append(position)

def cb_odom_lead(msg):
    position = msg.pose.pose.position
    point_list_lead.append(position)

def draw(event):
    draw_route()
    if len(point_list_follow) != 0:
        draw_odom_folow()
    if len(point_list_lead) != 0:
        draw_odom_lead()    

def draw_route():
    route_marker = Marker()
    route_marker.header.frame_id = "odom"
    route_marker.header.stamp = rospy.Time.now()
    route_marker.action = route_marker.ADD
    route_marker.type = route_marker.LINE_STRIP

    # maker scale
    route_marker.scale.x = 0.2
    route_marker.scale.y = 0.2
    route_marker.scale.z = 0.2

    # marker color
    route_marker.color.a = 1.0
    route_marker.color.r = 0.0
    route_marker.color.g = 0.0
    route_marker.color.b = 1.0

    # marker orientaiton
    route_marker.pose.orientation.x = 0.0
    route_marker.pose.orientation.y = 0.0
    route_marker.pose.orientation.z = 0.0
    route_marker.pose.orientation.w = 1.0

    # marker position
    route_marker.pose.position.x = 0.0
    route_marker.pose.position.y = 0.0
    route_marker.pose.position.z = 0.0

    route_marker.lifetime = rospy.Duration()

    for point in route_list:
        x = point[0]
        y = point[1]
        pt = Point(x, y, 0)
        route_marker.points.append(pt)

    pub_route_line.publish(route_marker)

def draw_odom_folow():
    
    line_marker = Marker()
    line_marker.header.frame_id = "odom"
    line_marker.header.stamp = rospy.Time.now()
    line_marker.action = line_marker.ADD
    line_marker.type = line_marker.LINE_STRIP

    # maker scale
    line_marker.scale.x = 0.15
    line_marker.scale.y = 0.15
    line_marker.scale.z = 0.15

    # marker color
    line_marker.color.a = 0.8
    line_marker.color.r = 1.0
    line_marker.color.g = 0.0
    line_marker.color.b = 0.0

    # marker orientaiton
    line_marker.pose.orientation.x = 0.0
    line_marker.pose.orientation.y = 0.0
    line_marker.pose.orientation.z = 0.0
    line_marker.pose.orientation.w = 1.0

    # marker position
    line_marker.pose.position.x = 0.0
    line_marker.pose.position.y = 0.0
    line_marker.pose.position.z = 0.0
        
    for point in point_list_follow:
        x = point.x
        y = point.y
        pt = Point(x, y, 0)
        line_marker.points.append(pt)
    pub_odom_line_follow.publish(line_marker)

def draw_odom_lead():
    
    line_marker = Marker()
    line_marker.header.frame_id = "odom"
    line_marker.header.stamp = rospy.Time.now()
    line_marker.action = line_marker.ADD
    line_marker.type = line_marker.LINE_STRIP

    # maker scale
    line_marker.scale.x = 0.18
    line_marker.scale.y = 0.18
    line_marker.scale.z = 0.18

    # marker color
    line_marker.color.a = 1.0
    line_marker.color.r = 0.0
    line_marker.color.g = 0.0
    line_marker.color.b = 0.0

    # marker orientaiton
    line_marker.pose.orientation.x = 0.0
    line_marker.pose.orientation.y = 0.0
    line_marker.pose.orientation.z = 0.0
    line_marker.pose.orientation.w = 1.0

    # marker position
    line_marker.pose.position.x = 0.0
    line_marker.pose.position.y = 0.0
    line_marker.pose.position.z = 0.0
        
    for point in point_list_lead:
        x = point.x
        y = point.y
        pt = Point(x, y, 0)
        line_marker.points.append(pt)
    pub_odom_line_lead.publish(line_marker)

def cb_srv_clear(req):
    del point_list_follow[:]
    del point_list_lead[:]
    print("Clear point")
    value = Int32()
    value = 0
    pub_clear.publish(value)
    return TriggerResponse()

if __name__ == "__main__":
    rospy.init_node("odometry_plot",anonymous=False)

    srv_clear = rospy.Service('~clear', Trigger, cb_srv_clear)
    
    #Circle
    #route_list = [(11,9),(5,6),(1,2),(-2,-4),(-2,-10),(1,-16),(5,-20),(11,-23),(17,-23),(23,-20),(27,-16),(30,-10),(30,-4),(27,2),(23,6),(17,9),(11,9)]

    # Square
    route_list = [(0, 7), (0, -21), (28, -21), (28, 7), (0, 7)]

    # Eight
    #route_list = [(7,-14),(7,0),(21,0),(21,-14),(35,-14),(35,0),(21,0),(21,-14),(7,-14)]

    #rospy.Subscriber("/MONICA/localization_gps_imu/odometry", Odometry, cb_odom_follow, queue_size=1)
    #rospy.Subscriber("/BRIAN/localization_gps_imu/odometry", Odometry, cb_odom_lead, queue_size=1)

    rospy.Subscriber("/MONICA/p3d_odom", Odometry, cb_odom_follow, queue_size=1)
    rospy.Subscriber("/BRIAN/p3d_odom", Odometry, cb_odom_lead, queue_size=1)

    pub_route_line = rospy.Publisher("/route_linelist", Marker, queue_size=1)

    pub_odom_line_follow = rospy.Publisher("/odom_linelist_follow", Marker, queue_size=1)
    pub_odom_line_lead = rospy.Publisher("/odom_linelist_lead", Marker, queue_size=1)

    pub_clear = rospy.Publisher("/clear", Int32, queue_size=1)
    
    rospy.Timer(rospy.Duration(1.0), draw)

    rospy.spin()