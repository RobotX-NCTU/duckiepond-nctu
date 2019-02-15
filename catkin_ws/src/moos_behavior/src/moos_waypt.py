#!/usr/bin/env python
'''
Author: Tony Hsiao                                                              
Date:2019/01/06                                                                
Last update: 2019/01/06                                                         
moos waypoint bhv 
'''
import rospy
#from duckiepond_vehicle.msg import UsvDrive
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
import tf
import math
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty
from duckiepond.srv import SetValue, SetValueRequest, SetValueResponse
from duckiepond.msg import MotorCmd

class MOOSWaypt(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name)) 

        # Param
        self.sim  = rospy.get_param('~sim', True)
        self.linear_speed  = rospy.get_param('~linear_speed', 0.0)
        self.angular_speed  = rospy.get_param('~angular_speed', 0.0)
        self.vname = rospy.get_param("~vname", None)

        # Variables
        self.wamv_pose = None   # geometry_msgs/Pose
        self.wamv_desired_speed = 0.0
        self.wamv_desired_rudder = 0.0
        self.wamv_desired_thruster_l = 0.0
        self.wamv_desired_thruster_r = 0.0
        self.wamv_desired_heading = None
        self.dbtime = None    

        # Ros service
        self.srv_linear_speed = rospy.Service('~set_linear', SetValue, self.cb_srv_set_linear_speed)
        self.srv_angular_speed = rospy.Service('~set_angular', SetValue, self.cb_srv_set_angular_speed)
        self.srv_reset = rospy.Service('~reset', SetValue, self.cb_srv_reset)


        ####################################################################################
        #### ROS Publisher
        ####################################################################################

        # To moos
        self.pub_nav_x      = rospy.Publisher("~NAV_X", Float64, queue_size=1)
        self.pub_nav_y      = rospy.Publisher("~NAV_Y", Float64, queue_size=1)
        self.pub_nav_heading   = rospy.Publisher("~NAV_HEADING", Float64, queue_size=1)
        self.pub_nav_speed   = rospy.Publisher("~NAV_SPEED", Float64, queue_size=1)

        # To ros
        if self.sim:
            from duckiepond_vehicle.msg import UsvDrive
            self.pub_motion = rospy.Publisher("~cmd_drive", UsvDrive, queue_size=1)
        else:
            self.pub_motion = rospy.Publisher("~cmd_drive", MotorCmd, queue_size=1)

        # Timer
        rospy.Timer(rospy.Duration(0.2), self.publish_data_to_moos)
        rospy.Timer(rospy.Duration(0.2), self.send_motor_cmd)


        ####################################################################################
        #### ROS Subscriber
        ####################################################################################

        # From ros
        self.sub_odom = rospy.Subscriber("~odometry", Odometry, self.cb_odom)
        
        # From moos
        self.sub_dbtime = rospy.Subscriber("~DB_TIME", Float64, self.cb_time)
        self.sub_desired_rudder = rospy.Subscriber("~DESIRED_RUDDER", Float64, self.cb_rudder)
        self.sub_desired_speed = rospy.Subscriber("~DESIRED_SPEED", Float64, self.cb_speed)
        self.sub_desired_heading = rospy.Subscriber("~DESIRED_HEADING", Float64, self.cb_heading)
        self.sub_desired_heading = rospy.Subscriber("~DESIRED_THRUST_L", Float64, self.cb_thruster_l)
        self.sub_desired_heading = rospy.Subscriber("~DESIRED_THRUST_R", Float64, self.cb_thruster_r)

    def send_motor_cmd(self, event):
        motor_msg = None
        if self.sim:
            from duckiepond_vehicle.msg import UsvDrive
            motor_msg = UsvDrive()

        else:
            motor_msg = MotorCmd()
            motor_msg.header.stamp = rospy.Time.now()
            
        motor_msg.left = 0
        motor_msg.right = 0
        if self.wamv_desired_heading is None or self.wamv_desired_rudder is None or self.wamv_desired_speed is None:
            pass
        elif abs(self.wamv_desired_rudder-0.0)<0.001 and abs(self.wamv_desired_thruster_l ) < 5.0 and self.wamv_desired_speed < 0.1:    
            pass
        else:
            if abs(self.wamv_desired_thruster_l ) < 5.0 :
                motor_msg.left = self.linear_speed 
                motor_msg.right = self.linear_speed
             
            if abs(self.wamv_desired_speed-0)<0.002 and abs(self.wamv_desired_thruster_l ) > 5.0:
                motor_msg.left = motor_msg.left + self.wamv_desired_thruster_l * 4 * self.angular_speed
                motor_msg.right = motor_msg.right + self.wamv_desired_thruster_r * 4 * self.angular_speed
                
            else:
                motor_msg.left = motor_msg.left + self.wamv_desired_rudder  * self.angular_speed * self.wamv_desired_speed
                motor_msg.right = motor_msg.right - self.wamv_desired_rudder * self.angular_speed * self.wamv_desired_speed                

        if motor_msg.left >= 0.9:
            motor_msg.left = 0.9
        elif motor_msg.left <= -0.9:
            motor_msg.left = -0.9

        if motor_msg.right >= 0.9:
            motor_msg.right = 0.9
        elif motor_msg.right <= -0.9:
            motor_msg.right = -0.9

        self.pub_motion.publish(motor_msg)


    def publish_data_to_moos(self, event):
        ####################################################################################
        #### Send all informations to MOOS
        #################################################################################### 
        if self.dbtime is not None and self.wamv_pose is not None:
            #rospy.loginfo("%s send data." %(self.node_name))

            nav_x = Float64()
            nav_x.data = self.wamv_pose.position.x
            self.pub_nav_x.publish(nav_x)   

            nav_y = Float64()
            nav_y.data = self.wamv_pose.position.y 
            self.pub_nav_y.publish(nav_y)  

            nav_heading = Float64()
            nav_heading.data = (90 - self.quaternion_to_yaw(self.wamv_pose.orientation) / math.pi * 180) % 360
            self.pub_nav_heading.publish(nav_heading)      

            nav_speed = Float64()
            nav_speed.data = self.linear_speed
            self.pub_nav_speed.publish(nav_speed)

    def cb_thruster_l(self, msg):
        self.wamv_desired_thruster_l = msg.data

    def cb_thruster_r(self, msg):
        self.wamv_desired_thruster_r = msg.data   

    def cb_heading(self, msg):
        self.wamv_desired_heading = msg.data

    def cb_odom(self, msg):
        self.wamv_pose = msg.pose.pose
        
    def cb_time(self, msg):
        self.dbtime = msg.data

    def cb_rudder(self, msg):
        self.wamv_desired_rudder = msg.data

    def cb_speed(self, msg):
        self.wamv_desired_speed = msg.data
    
    def quaternion_to_yaw(self, qua):
        q=[qua.x, qua.y, qua.z, qua.w]
        _   , _, yaw = tf.transformations.euler_from_quaternion(q) 
        return yaw


    def cb_srv_set_linear_speed(self, req):
        self.linear_speed = req.value
        rospy.loginfo("%s Set Linear Speed %f." %(self.vname, self.linear_speed))
        return SetValueResponse()

    def cb_srv_set_angular_speed(self, req):
        self.angular_speed = req.value
        rospy.loginfo("%s Set Angular Speed %f." %(self.vname, self.angular_speed))
        return SetValueResponse()

    def cb_srv_reset(self, req):
        self.wamv_desired_heading = None 
        self.wamv_desired_rudder = None
        return SetValueResponse()  


    def create_color_feature_string(self, x, y):
        ####################################################################################
        #### Create moos data about obstacles informations
        ####################################################################################    
            
        color_featured = "x=" + str(x) + ", y="+ str(y) + ", size=1, color=red"
        return color_featured

    def on_shutdown(self):

        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('moos_waypt_node', anonymous=False)
    moos_waypt_node = MOOSWaypt()
    rospy.on_shutdown(moos_waypt_node.on_shutdown)
    rospy.spin()
