#!/usr/bin/env python
'''
Author: Tony Hsiao                                                              
Date:2019/01/16                                                                
Last update: 2019/01/16                                                         
Locailization by gps and imu
'''
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry, Pose
from std_msgs.msg import Float64
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty
from duckiepond.srv import SetValue, SetValueRequest, SetValueResponse

from geodesy.utm import UTMPoint, fromLatLong
import tf
import math
from scipy.stats import norm

class LocailizationGPSImu(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.pose = Pose()
        self.prior_pose = Pose()
        self.first_time = True   
        
        # param
        self.imu_offset = 0
        self.lat_orig = rospy.get_param('~latitude', 0.0)
        self.long_orig = rospy.get_param('~longitude', 0.0)
        self.utm_orig = fromLatLong(self.lat_orig, self.long_orig)

        # Service
        self.srv_imu_offset = rospy.Service('~imu_offset', SetValue, self.cb_srv_imu_offest)

        # Publisher
        self.pub_odm = rospy.Publisher("~odometry", Odometry, queue_size=1)

        # Subscriber
        sub_imu = message_filters.Subscriber("~imu/data", Imu)
        sub_gps = message_filters.Subscriber("~fix", NavSatFix)
        ats = ApproximateTimeSynchronizer((sub_imu, sub_gps), queue_size = 1, slop = 0.1)
        ats.registerCallback(self.cb_gps_imu)

    def cb_gps(self, msg_gps):
        utm_point = fromLatLong(msg.latitude, msg.longitude)
        self.pose.position.x = utm_point.easting - self.utm_orig.easting
        self.pose.position.y = utm_point.northing - self.utm_orig.northing


    def cb_imu(self, msg_imu):
        self.pose.orientation = msg_imu.orientation

    def cb_gps_imu(self, msg_imu, msg_gps):
        self.cb_gps(msg_gps)
        self.cb_imu(msg_imu)
        self.kalman_filter()

    def kalman_filter(self):
        if self.first_time == True:
            self.first_time = False
            self.prior_pose = self.pose



    def cb_srv_imu_offest(self, request): 
        self.imu_offset = request.data
        print ("Set imu offset = " + str(self.imu_offset))
        return SetFloatResponse()

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('localization_gps_imu_node',anonymous=False)
    localization_gps_imu_node = LocailizationGPSImu()
    rospy.on_shutdown(localization_gps_imu_node.onShutdown)
    rospy.spin()

