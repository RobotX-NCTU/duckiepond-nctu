#!/usr/bin/env python
import numpy as np
import cv2
import roslib
import rospy
import tf
import struct
import math
import time
from sensor_msgs.msg import Image, LaserScan
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from control.cfg import pos_PIDConfig, ang_PIDConfig
from duckiepond.msg import Boxlist, MotorCmd, UsvDrive
from std_srvs.srv import SetBool, SetBoolResponse
from PID import PID_control


class Tracking():
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " % (self.node_name))
        self.ROBOT_NUM = 3
        self.wavm_labels = ["wamv", ""]
        # rospy.Subscriber('/pcl_points_img', PoseArray, self.call_back, queue_size = 1, buff_size = 2**24)
        self.combine_moos = rospy.get_param("Tracking/combine_moos", False)

        # Image definition
        self.width = 640
        self.height = 480
        self.h_w = 10.
        self.const_SA = 0.7
        self.bridge = CvBridge()
        self.predict_prob = 0.5

        self.pos_ctrl_max = 1
        self.pos_ctrl_min = -1
        self.cmd_ctrl_max = 0.95
        self.cmd_ctrl_min = -0.95
        self.station_keeping_dis = 3.5  # meters
        self.frame_id = 'odom'
        self.is_station_keeping = False
        self.stop_pos = []
        self.final_goal = None  # The final goal that you want to arrive
        self.goal = self.final_goal
        self.tracking_id = 0

        rospy.loginfo("[%s] Initializing " % (self.node_name))
        self.sim = rospy.get_param("Tracking/sim", False)
        # self.image_sub = rospy.Subscriber("/BRIAN/camera_node/image/compressed", Image, self.img_cb, queue_size=1)
        self.image_sub = rospy.Subscriber(
            "detecter/predictions", Boxlist, self.box_cb, queue_size=1, buff_size=2**24)
        publisher_name = "cmd_drive"
        if self.combine_moos:
            publisher_name = "ssd_drive"
        if self.sim:
            from duckiepond_vehicle.msg import UsvDrive
            self.pub_cmd = rospy.Publisher(
                publisher_name, UsvDrive, queue_size=1)
            self.cmd_msg = UsvDrive()
        else:
            self.pub_cmd = rospy.Publisher(
                publisher_name, MotorCmd, queue_size=1)
            self.cmd_msg = MotorCmd()
        self.pub_goal = rospy.Publisher("goal_point", Marker, queue_size=1)
        self.image_pub = rospy.Publisher(
            "motion_img/compressed", CompressedImage, queue_size=1)
        self.station_keeping_srv = rospy.Service(
            "station_keeping", SetBool, self.station_keeping_cb)

        self.pos_control = PID_control("Position_tracking")
        self.ang_control = PID_control("Angular_tracking")

        self.ang_station_control = PID_control("Angular_station")
        self.pos_station_control = PID_control("Position_station")

        self.pos_srv = Server(
            pos_PIDConfig, self.pos_pid_cb, "Position_tracking")
        self.ang_srv = Server(
            ang_PIDConfig, self.ang_pid_cb, "Angular_tracking")
        self.pos_station_srv = Server(
            pos_PIDConfig, self.pos_station_pid_cb, "Angular_station")
        self.ang_station_srv = Server(
            ang_PIDConfig, self.ang_station_pid_cb, "Position_station")

        self.initialize_PID()

        self.timer = rospy.Timer(rospy.Duration(0.2), self.cb_publish)

    def cb_publish(self, event):
        self.pub_cmd.publish(self.cmd_msg)

    def box_cb(self, msg):
        self.width = msg.image_width
        self.height = msg.image_height
        try:
            np_arr = np.fromstring(msg.image.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        boxes = msg.list
        predict = self.get_control_info(boxes, cv_image)
        if predict is None:
            return
        angle, dis = predict[0], predict[1]
        self.tracking_control(angle, dis)

    def tracking_control(self, goal_angle, goal_distance):
        if self.is_station_keeping:
            rospy.loginfo("Station Keeping")
            pos_output, ang_output = self.station_keeping(
                goal_distance, goal_angle)
        else:
            pos_output, ang_output = self.control(goal_distance, goal_angle)

        if self.sim:
            self.cmd_msg = UsvDrive()
        else:
            self.cmd_msg = MotorCmd()
        # print(cmd_msg.left,cmd_msg.right)
        self.cmd_msg.left = self.cmd_constarin(pos_output - ang_output)
        self.cmd_msg.right = self.cmd_constarin(pos_output + ang_output)
        print(self.cmd_msg.left, self.cmd_msg.right)

        # self.publish_goal(self.goal)

    def get_control_info(self, boxes, img):
        # Image Preprocessing (vgg use BGR image as training input)
        if len(boxes) == 0:
            return

        bbox = None
        for box in boxes:
            if box.id == self.tracking_id:
                bbox = box
        if bbox is None:
            bbox = boxes[0]
            self.tracking_id == bbox.id

        angle, dis, center = self.BBx2AngDis(bbox)
        cv2.circle(img, (int(center[0]), int(center[1])), 10, (0, 0, 255), -1)
        cv2.rectangle(img, (int(bbox.x), int(bbox.y)),
                      (int(bbox.x + bbox.w), int(bbox.y + bbox.h)), (0, 0, 255), 5)
        try:
            img = self.draw_cmd(img, dis, angle)
            img_msg = self.bridge.cv2_to_compressed_imgmsg(img)
            img_msg.header = Header()
            img_msg.header.stamp = rospy.Time.now()
            self.image_pub.publish(img_msg)
        except CvBridgeError as e:
            print(e)
        return angle, dis

    def draw_cmd(self, img, dis, angle):
        v = dis
        omega = angle
        rad = omega*math.pi/2.  # [-1.57~1.57]
        rad = rad - math.pi/2.  # rotae for correct direction
        radius = 120
        alpha = 0.3
        v_length = radius*math.sqrt(v**2 + v**2)/math.sqrt(1**2 + 1**2)
        if v_length > radius:
            v_length = radius
        x = v_length*math.cos(rad)
        y = v_length*math.sin(rad)
        x_max = radius*math.cos(rad)
        y_max = radius*math.sin(rad)
        center = (int(self.width/2.), int(self.height))
        draw_img = img.copy()
        cv2.circle(draw_img, (center), radius, (255, 255, 255), 10)
        cv2.addWeighted(draw_img, alpha, img, 1 - alpha, 0, img)
        # cv2.circle(img, (int(center[0]+x_max), int(center[1]+y_max)), 15, (255, 255, 255), -1)
        cv2.arrowedLine(
            img, center, (int(center[0]+x), int(center[1]+y)), (255, 255, 255), 7)
        return img

    def BBx2AngDis(self, bbox):
        x = bbox.x
        y = bbox.y
        w = bbox.w
        h = bbox.h
        center = (x + w/2., y + h/2.)
        angle = (center[0]-self.width/2.)/(self.width/2.)
        dis = float((self.height - h))/(self.height)
        return angle, dis, center

    def control(self, goal_distance, goal_angle):
        self.pos_control.update(5*(goal_distance - self.const_SA))
        self.ang_control.update(goal_angle)

        # pos_output will always be positive
        pos_output = -self.pos_constrain(self.pos_control.output)

        # -1 = -180/180 < output/180 < 180/180 = 1
        ang_output = self.ang_control.output
        return pos_output, ang_output

    def station_keeping(self, goal_distance, goal_angle):
        self.pos_station_control.update(goal_distance)
        self.ang_station_control.update(goal_angle)

        # pos_output will always be positive
        pos_output = self.pos_station_constrain(
            -self.pos_station_control.output/self.dis4constV)

        # -1 = -180/180 < output/180 < 180/180 = 1
        ang_output = self.ang_station_control.output/180.

        # if the goal is behind the robot
        if abs(goal_angle) > 90:
            pos_output = - pos_output
            ang_output = - ang_output
        return pos_output, ang_output

    def station_keeping_cb(self, req):
        if req.data == True:
            self.goal = self.stop_pos
            self.is_station_keeping = True
        else:
            self.goal = self.final_goal
            self.is_station_keeping = False
        res = SetBoolResponse()
        res.success = True
        res.message = "recieved"
        return res

    def cmd_constarin(self, input):
        if input > self.cmd_ctrl_max:
            return self.cmd_ctrl_max
        if input < self.cmd_ctrl_min:
            return self.cmd_ctrl_min
        return input

    def pos_constrain(self, input):
        if input > self.pos_ctrl_max:
            return self.pos_ctrl_max
        if input < self.pos_ctrl_min:
            return self.pos_ctrl_min
        return input

    def initialize_PID(self):
        self.pos_control.setSampleTime(1)
        self.ang_control.setSampleTime(1)
        self.pos_station_control.setSampleTime(1)
        self.ang_station_control.setSampleTime(1)

        self.pos_control.SetPoint = 0.0
        self.ang_control.SetPoint = 0.0
        self.pos_station_control.SetPoint = 0.0
        self.ang_station_control.SetPoint = 0.0

    def get_goal_angle(self, robot_yaw, robot, goal):
        robot_angle = np.degrees(robot_yaw)
        p1 = [robot[0], robot[1]]
        p2 = [robot[0], robot[1]+1.]
        p3 = goal
        angle = self.get_angle(p1, p2, p3)
        result = angle - robot_angle
        result = self.angle_range(-(result + 90.))
        return result

    def get_angle(self, p1, p2, p3):
        v0 = np.array(p2) - np.array(p1)
        v1 = np.array(p3) - np.array(p1)
        angle = np.math.atan2(np.linalg.det([v0, v1]), np.dot(v0, v1))
        return np.degrees(angle)

    # limit the angle to the range of [-180, 180]
    def angle_range(self, angle):
        if angle > 180:
            angle = angle - 360
            angle = self.angle_range(angle)
        elif angle < -180:
            angle = angle + 360
            angle = self.angle_range(angle)
        return angle

    def get_distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def publish_goal(self, goal):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pure_pursuit"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.orientation.w = 1
        marker.pose.position.x = goal[0]
        marker.pose.position.y = goal[1]
        marker.id = 0
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 0.6
        marker.color.a = 1.0
        marker.color.g = 1.0
        self.pub_goal.publish(marker)

    def pos_pid_cb(self, config, level):
        print(
            "Position: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
        Kp = float("{Kp}".format(**config))
        Ki = float("{Ki}".format(**config))
        Kd = float("{Kd}".format(**config))
        self.pos_control.setKp(Kp)
        self.pos_control.setKi(Ki)
        self.pos_control.setKd(Kd)
        return config

    def ang_pid_cb(self, config, level):
        print(
            "Angular: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
        Kp = float("{Kp}".format(**config))
        Ki = float("{Ki}".format(**config))
        Kd = float("{Kd}".format(**config))
        self.ang_control.setKp(Kp)
        self.ang_control.setKi(Ki)
        self.ang_control.setKd(Kd)
        return config

    def pos_station_pid_cb(self, config, level):
        print(
            "Position: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
        Kp = float("{Kp}".format(**config))
        Ki = float("{Ki}".format(**config))
        Kd = float("{Kd}".format(**config))
        self.pos_station_control.setKp(Kp)
        self.pos_station_control.setKi(Ki)
        self.pos_station_control.setKd(Kd)
        return config

    def ang_station_pid_cb(self, config, level):
        print(
            "Angular: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
        Kp = float("{Kp}".format(**config))
        Ki = float("{Ki}".format(**config))
        Kd = float("{Kd}".format(**config))
        self.ang_station_control.setKp(Kp)
        self.ang_station_control.setKi(Ki)
        self.ang_station_control.setKd(Kd)
        return config


if __name__ == '__main__':
    rospy.init_node('Tracking')
    foo = Tracking()
    rospy.spin()
