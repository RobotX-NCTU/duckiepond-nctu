#!/usr/bin/env python
import serial
import time
import rospy
import math
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
imu_PoseStamped = PoseStamped()

'''def imu_pose(imuMsg):
    imu_PoseStamped.header.stamp = imuMsg.header.stamp
    imu_PoseStamped.header.frame_id = "imu"
    imu_PoseStamped.pose = Pose(Point(0., 0., 0.), Quaternion(imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w))
    pose_pub.publish(imu_PoseStamped)'''

def imuCB(msg):
    imu_PoseStamped.header.stamp = msg.header.stamp
    imu_PoseStamped.header.frame_id = "imu"
    imu_PoseStamped.pose = Pose(Point(0., 0., 0.), Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))
    print "successful"
    pose_pub.publish(imu_PoseStamped)

if __name__ == '__main__':
    #init publisher
    rospy.init_node('imu_info_pub')
    pose_pub = rospy.Publisher("/imu_pose", PoseStamped, queue_size=1)
    sub_imu = rospy.Subscriber("/TONY/imu/data", Imu, imuCB, queue_size=1)
    rospy.spin()
