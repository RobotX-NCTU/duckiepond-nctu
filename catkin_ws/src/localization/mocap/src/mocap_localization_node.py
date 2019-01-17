#!/usr/bin/env python
import rospy
import numpy as np
import scipy as sp
import math
from apriltags2_ros.msg import AprilTagDetectionArray
from apriltags2_ros.msg import AprilTagDetection
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point, PoseStamped, Vector3
from nav_msgs.msg import Odometry, Path
from tf.transformations import quaternion_from_euler, euler_from_quaternion, compose_matrix, rotation_matrix
from visualization_msgs.msg import Marker, MarkerArray


class MocapLocalizationNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # visualization
        self.verbose = rospy.get_param("~verbose", False)
        self.simulation = rospy.get_param("~simulation", True)

        # get multiple robot names
        self.robot_names = rospy.get_param("~robot_names", "duckiepond_1,duckiepond_2")

        # base tag id~system_number
        self.system_number = rospy.get_param("~system_number", 1)

        # for fix mapping matric
        self.get_mapping_matrix = False
        self.T = []

        self.setup_tag_id()

        self.setup_transformation_real()

        # legal or illegal localization
        self.base_tag_detect_count = 0
        self.vehicle_tag_detect_count = 0

        # previous position of vehicle
        self.pre_vehicle_loalization = Point()
        self.count = 0

        self.setup_rviz_marker()

        # apriltag localization path
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"

        # multiple robot odomertry publisher
        self.pub_robot_odom = list()
        self.robot_topics = self.robot_names.split(',')

        # Subscribers
        self.sub_tag_detections = rospy.Subscriber("~tag_detections", AprilTagDetectionArray, self.processTagDetections, queue_size=10)
        
        # Publishers
        self.pub_vehicle_pose = rospy.Publisher("~als_posestamped", PoseStamped, queue_size=10)
        self.pub_visual_apriltag_wp = rospy.Publisher("~apriltag_cube_waypoint", MarkerArray, queue_size=10)
        for i in range(int(len(self.vehicle_tag_id)*0.5)):
            rostopic = '/' + self.robot_topics[i] + '/mocap_localization_node/odometry'
            self.pub_robot_odom.append(rospy.Publisher(rostopic, Odometry, queue_size=10))

        #self.pub_obstacle = rospy.Publisher("~obstacle", MarkerArray, queue_size=20)
        #self.pub_obstacle = rospy.Publisher("~obstacle", MarkerArray, queue_size=20)
        # self.pub_odom = rospy.Publisher('~tag_localization_odometry', Odometry, queue_size = 20)
        # self.pub_path = rospy.Publisher('~tag_localization_path', Path, queue_size = 20)

    def setup_tag_id(self):
        # base tag id
        if self.simulation:      
            self.base_tag_id_group = [[502, 504, 501, 503],[508, 509, 510, 511]]
            self.base_tag_id = self.base_tag_id_group[self.system_number-1]
        else:
            self.base_tag_id_group = [[502, 504, 501, 503],[508, 509, 510, 511]]
            self.base_tag_id = self.base_tag_id_group[self.system_number-1]
        # vehicle tag id    
        self.vehicle_tag_id = [509, 510, 511, 512]

    def setup_transformation_real(self):
        # transformation for real data
        angles = [0, 0, -0.68 * math.pi ]
        trans = [-7, 7, 0]
        self.M0 = compose_matrix(scale=None, shear=None, angles=angles, translate=trans, perspective=None)

        # shift bwtween tags and center of wamv
        self.shift_center = np.array([[0.5, -0.5, 0, 0], [0.95, 0, 0, 0], [0.5, 0.5, 0, 0], [1.1, 0.0, -0.9, 0]], dtype='f')
        self.shift_phi = np.array([math.pi*0.5, math.pi, math.pi, 0], dtype='f')
        self.vehicle_theta_pre = np.zeros((4, 1), dtype='f')

    def setup_rviz_marker(self):
        # apriltag cube and obstacles marker
        if self.simulation == True:
            self.apriltag_cube_point = np.array([[0, 0, 0.7], [20, 0, 0.7], [0, 20, 0.7], [20, 20, 0.7]], dtype='f')
            self.obs_sphere_point = []
        else:
            self.apriltag_cube_point = np.array([[14, 10, 0.7], [23, 10, 0.7], [13.5, 23.5, 0.7], [29, 22, 0.7]], dtype='f')
            self.obs_sphere_point = np.array([[-2.5, -11, 0.7], [-9, -24, 0.7], [-1.5, -21, 0.7]], dtype='f')

            #for i in range(4):
            #    origin_point = [self.apriltag_cube_point[i][0], self.apriltag_cube_point[i][1], self.apriltag_cube_point[i][2], 1]
            #    origin_after = self.M0.dot(origin_point)
            #    self.apriltag_cube_point[i][0] = origin_after[0]
            #    self.apriltag_cube_point[i][1] = origin_after[1]
            #    self.apriltag_cube_point[i][2] = origin_after[2]
            #print self.apriltag_cube_point

        self.marker_array = MarkerArray()

        self.marker = Marker()
        self.marker.header.frame_id = 'odom'
        self.marker.id = 0
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD
        self.marker.scale.x = 1
        self.marker.scale.y = 1
        self.marker.scale.z = 1
        self.marker.color.r = 1
        self.marker.color.g = 1
        self.marker.color.b = 1
        self.marker.color.a = 0.5

    def processTagDetections(self,tag_detections_msg):
        # assign base tag coordination
        # self.base_tag_point = np.array([[0, 0, 0.7], [20, 0, 0.7], [0, 20, 0.7], [20, 20, 0.7]], dtype='f') # for gazebo
        # self.base_tag_point = np.array([[0, 0, 0.7], [10.59, 0, 0.7], [0, 12.55, 0.7], [16.45, 13.934, 0.7]], dtype='f') # for bamboolake 20181117
        self.base_tag_point = self.apriltag_cube_point # after real transformation
        
        # Publish rviz marker
        for p in self.base_tag_point:
            self.marker.pose.position = Vector3(p[0], p[1], p[2])
            self.marker.id += 1
            self.marker.type = Marker.CUBE
            self.marker.color.r = 1
            self.marker.color.g = 1
            self.marker.color.b = 1
            self.marker_array.markers.append(self.marker)
            self.pub_visual_apriltag_wp.publish(self.marker_array)

        for p in self.obs_sphere_point:
            self.marker.pose.position = Vector3(p[0], p[1], p[2])
            self.marker.id += 1
            self.marker.type = Marker.SPHERE
            self.marker.color.r = 0
            self.marker.color.g = 1
            self.marker.color.b = 0
            self.marker_array.markers.append(self.marker)
            self.pub_visual_apriltag_wp.publish(self.marker_array)

        
        self.vehicle_tag_point_pair = np.zeros((4, 4), dtype='f')

        #self.vehicle_theta = np.zeros((4, 1), dtype='f')
        #self.vehicle_check = np.zeros((4, 1), dtype='f')

        self.obser_tag_point = np.zeros((4, 3), dtype='f')
        self.test_tag_point = np.zeros((4, 4), dtype='f')

        # assign header
        self.header = tag_detections_msg.header

        for tag_detection in tag_detections_msg.detections:
            # extract base tag detection
            for index, tag_id in enumerate(self.base_tag_id):
                if tag_detection.id[0] == tag_id:
                    self.obser_tag_point[index, 0] = tag_detection.pose.pose.pose.position.z
                    self.obser_tag_point[index, 1] = tag_detection.pose.pose.pose.position.x                   
                    self.obser_tag_point[index, 2] = tag_detection.pose.pose.pose.position.y
                    self.test_tag_point[index, 0] = tag_detection.pose.pose.pose.position.z
                    self.test_tag_point[index, 1] = tag_detection.pose.pose.pose.position.x                  
                    self.test_tag_point[index, 2] = tag_detection.pose.pose.pose.position.y
                    self.test_tag_point[index, 3] = 1
                    self.base_tag_detect_count += 1
                    #print "get base tag: ", tag_id
            # extract vehicle tag detection
            for index, tag_id in enumerate(self.vehicle_tag_id):
                if tag_detection.id[0] == tag_id:
                    a = tag_detection.pose.pose.pose.orientation
                    n = euler_from_quaternion([a.x, a.y, a.z, a.w]) 
                    #self.vehicle_theta[index, 0] = n[1]
                    #self.vehicle_check[index, 0] = abs(n[1])
                    self.vehicle_tag_point_pair[index, 0] = tag_detection.pose.pose.pose.position.z
                    self.vehicle_tag_point_pair[index, 1] = tag_detection.pose.pose.pose.position.x               
                    self.vehicle_tag_point_pair[index, 2] = tag_detection.pose.pose.pose.position.y
                    self.vehicle_tag_point_pair[index, 3] = 1
                    self.vehicle_tag_detect_count += 1 
                    print "get vehicle tag: ", tag_id
                    print n

        # check whether two pitch of adjacent tag detections are half of pi
        #if self.vehicle_tag_detect_count == 2:
        #    theta_sum = self.vehicle_check.sum(axis=0)
        #    print "sum of theta: ", theta_sum
        #    if abs(theta_sum - math.pi * 0.5) > 0.2:
        #        print "illegal detection"
        #        self.base_tag_detect_count = 0
        #        self.vehicle_tag_detect_count = 0
        #        return
        
        # check whether pitch variance of same tag detection is too sharp
        #for i in range(4):
        #    if((self.vehicle_theta[i, 0] * self.vehicle_theta_pre[i, 0]) < -0.4):
        #       print "too large varing yaw"
        #        self.base_tag_detect_count = 0
        #        self.vehicle_tag_detect_count = 0
        #        for j in range(4):
        #            a = self.vehicle_theta[i, 0]
        #            self.vehicle_theta_pre[i, 0] = a
        #        return
        #for j in range(4):
        #    a = self.vehicle_theta[i, 0]
        #    self.vehicle_theta_pre[i, 0] = a
        #if(self.verbose): print self.vehicle_theta
        #if(self.verbose): print self.shift_phi

        # check enough tags detected
        #if(self.verbose): print 'system: ', self.system_number
        #print 'base tag count:',self.base_tag_detect_count 
        #print 'vehicle tag count:',self.vehicle_tag_detect_count
        if self.get_mapping_matrix == False: # not yet get mapping matrix                   
            if(self.base_tag_detect_count < 4 or self.vehicle_tag_detect_count == 0):# non enough base or vehicle tag detected
                self.base_tag_detect_count = 0
                self.vehicle_tag_detect_count = 0
                if(self.verbose): print 'non enough tags detectecd'
                rospy.loginfo("non enough tags detectecd")
                return
        else: # get mapping matrix                   
            if(self.vehicle_tag_detect_count == 0):# non enough vehicle tag detected
                self.base_tag_detect_count = 0
                self.vehicle_tag_detect_count = 0
                if(self.verbose): print 'non enough tags detectecd'
                rospy.loginfo("non enough tags detectecd")
                return                

        # vehicle tag detection shift compensation
        #for i in range(4):
        #    if self.vehicle_tag_point_pair[i,2] != 0:
        #        dx = (-1 * abs(self.shift_center[i,0]) * math.cos(math.pi+self.vehicle_theta[i,0]))
        #        dy = (1 * abs(self.shift_center[i,0]) * math.sin(math.pi+self.vehicle_theta[i,0]))
        #        if(self.verbose): print dx
        #        if(self.verbose): print dy
        #        self.vehicle_tag_point_pair[i, 0] += dx 
        #        self.vehicle_tag_point_pair[i, 1] -= dy        


        # remove zero row if only three base apriltag detected
        #if(self.base_tag_detect_count == 3):
        #    zero_row = np.where(~self.obser_tag_point.any(axis=1))[0][0]
        #    self.base_tag_point = np.delete(self.base_tag_point, zero_row, axis=0)
        #    self.obser_tag_point = self.obser_tag_point[~(self.obser_tag_point==0).all(1)]


        #print self.base_tag_point.transpose()
        #print self.obser_tag_point.transpose()
        #print self.vehicle_tag_point_pair.transpose()

        if(self.base_tag_detect_count == 4):
        #if self.get_mapping_matrix == False:
            length = self.obser_tag_point.shape[0]
            p_ct = self.base_tag_point.sum(axis=0) / length
            p_cm = self.obser_tag_point.sum(axis=0) / length

            self.obser_tag_point = self.obser_tag_point - p_cm
            self.base_tag_point = self.base_tag_point - p_cm

            Mtd = self.base_tag_point[0]
            Mmd = self.obser_tag_point[0]
            for i in range(length-1):
                Mtd = np.vstack((Mtd, self.base_tag_point[i+1]))
                Mmd = np.vstack((Mmd, self.obser_tag_point[i+1]))
            Mtd = Mtd.transpose()
            Mmd = Mmd.transpose()

            #p_ct = (self.base_tag_point[0] + self.base_tag_point[1] + self.base_tag_point[2])/3
            #p_cm = (self.obser_tag_point[0] + self.obser_tag_point[1] + self.obser_tag_point[2])/3
            #for i in range(3):
            #    self.obser_tag_point[i] = self.obser_tag_point[i] - p_cm
            #    self.base_tag_point[i] = self.base_tag_point[i] - p_ct
            #Mtd = np.vstack((self.base_tag_point[0], self.base_tag_point[1], self.base_tag_point[2])).transpose()
            #Mmd = np.vstack((self.obser_tag_point[0], self.obser_tag_point[1], self.obser_tag_point[2])).transpose()

            H = np.dot(Mmd, Mtd.transpose())
            [U, D, V] = np.linalg.svd(H,full_matrices=1)
            R = np.dot(V,U.transpose())
            t = np.matrix(p_ct - np.dot(R,p_cm))

            temp = np.hstack((R,t.transpose()))
            zero = np.array([[0,0,0,1]])
            T = np.vstack((temp,zero))
            self.T = T
            self.get_mapping_matrix =True

        self.base_tag_detect_count = 0
        self.vehicle_tag_detect_count = 0

        #print self.T
        vehicle_loalization = np.dot(self.T,self.vehicle_tag_point_pair.transpose())
        #print 'after transformation'
        #print np.dot(self.T,self.test_tag_point.transpose())
        #print vehicle_loalization
        #shift = self.shift_center.transpose()
        #for i in range(3):
        #    if vehicle_loalization[:,i][3] != 0:
        #        for j in range(4):
        #            vehicle_loalization[:,i][j] += shift[:,i][j]
        #            vehicle_loalization[:,i][j] = vehicle_loalization[:,i][j] 
        #if(self.verbose): print "after shift"
        #if(self.verbose): print vehicle_loalization
        #vehicle_loalization = vehicle_loalization.sum(axis=1)/self.vehicle_tag_detect_count
        #vehicle_loalization[2] = 0
        # "average", vehicle_loalization

        #self.base_tag_detect_count = 0
        #self.vehicle_tag_detect_count = 0

        odom_msg = Odometry()
        odom_msg.header = self.header
        odom_msg.header.frame_id = "/odom"

        vehicle_number = int(len(vehicle_loalization)*0.5)
        #print vehicle_loalization[0,0]
        #print vehicle_loalization[0,1]        
        for i in range(vehicle_number):
            # publish robot position
            x1 = vehicle_loalization[0, i*2 + 0]
            x0 = vehicle_loalization[0, i*2 + 1]
            y1 = vehicle_loalization[1, i*2 + 0]
            y0 = vehicle_loalization[1, i*2 + 1]	
            if(x1 == 0 or x0 == 0):
            	print 'only one tag'
            	continue
            odom_msg.pose.pose.position.x = (x1 + x0) * 0.5 
            odom_msg.pose.pose.position.y = (y1 + y0) * 0.5
            # publish robot orientation
            dx = x1 - x0
            dy = y1 - y0
            yaw = math.atan2(dy, dx)
            print i, ": ", yaw
            quan = quaternion_from_euler(0, 0, yaw)
            odom_msg.pose.pose.orientation.x = quan[0]
            odom_msg.pose.pose.orientation.y = quan[1]
            odom_msg.pose.pose.orientation.z = quan[2]
            odom_msg.pose.pose.orientation.w = quan[3]
            #print odom_msg.pose.pose.position 
            self.pub_robot_odom[i].publish(odom_msg)
        #vehicle_pose = Pose()
        #vehicle_pose.position.x = vehicle_loalization[0] 
        #vehicle_pose.position.y = vehicle_loalization[1] 
        #vehicle_pose.position.z = vehicle_loalization[2]
        #print vehicle_pose.position.x, vehicle_pose.position.y, vehicle_pose.position.z
        #print "average", vehicle_loalization
        # publish odometry
        #self.cb_odom(vehicle_pose.position)

        # publish path
        # self.cb_path(vehicle_pose)
        #print self.T
        # publish posestamped
        #self.cb_pose_stam(vehicle_pose)
        #self.count += 1
        #print 'total: ', self.count

        #self.pre_vehicle_loalization = vehicle_pose.position

    def cb_pose_stam(self, p):
        pose_stamped_msg = PoseStamped()


        pose_stamped_msg.header = self.header
        pose_stamped_msg.header.frame_id = "odom"

        pose_stamped_msg.pose = p

        self.pub_vehicle_pose.publish(pose_stamped_msg)
        if(self.verbose): print 'pose'
        if(self.verbose): print pose_stamped_msg.pose
        if(self.verbose): print '------------'

    #def cb_path(self, p):
    #    pose_stamped = PoseStamped()

        #self.path_msg.header.stamp = rospy.Time.now()
        #self.path_msg.header.seq = pose_stamped.header.seq
        #pose_stamped.header.stamp = self.path_msg.header.stamp
    #    pose_stamped.header = self.header
    #    pose_stamped.header.frame_id = "odom"

    #    pose_stamped.pose = p

    #    self.path_msg.poses.append(pose_stamped)
    #    #self.pub_path.publish(self.path_msg)
    #    print 'poses------------'
    #    print self.path_msg.poses
    #    print '------------'


    #def cb_odom(self, point):
    #    odom_msg = Odometry() 

    #    odom_msg.header = self.header
    #    # reassign frame_id
    #    odom_msg.header.frame_id = "odom"
    #    #odom_msg.header.stamp = rospy.Time.now()

    #    odom_msg.pose.pose.position = point
    #    #odom_msg.pose.pose.orientation = self.get_orientation(point)
    #    print "odem", odom_msg.pose.pose
    #    #self.pub_odom.publish(odom_msg)

    def get_orientation(self, point):
        dx = point.x - self.pre_vehicle_loalization.x
        dy = point.y - self.pre_vehicle_loalization.y
        quat_msg = Quaternion()
        self.quat = []
        if(dx ==0):
            self.quat = quaternion_from_euler (0, 0, math.pi*0.5)
        else:
            theta = math.atan2(dy, dx)
            if(theta < 0):
                theta += math.pi * 2
            self.quat = quaternion_from_euler (0, 0, theta)
        print dx, dy, self.quat
        quat_msg.x = self.quat[0]
        quat_msg.y = self.quat[1]
        quat_msg.z = self.quat[2]
        quat_msg.w = self.quat[3]
        return quat_msg


    def onShutdown(self):
        rospy.loginfo("[MocapLocalizationNode] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('mocap_localization_node',anonymous=False)
    mocap_localization_node = MocapLocalizationNode()
    rospy.on_shutdown(mocap_localization_node.onShutdown)
    rospy.spin()
