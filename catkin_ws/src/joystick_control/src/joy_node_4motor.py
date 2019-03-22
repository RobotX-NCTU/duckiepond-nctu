#!/usr/bin/env python
import rospy
import math
import numpy as np
import tf
from sensor_msgs.msg import Joy,Imu
from duckiepond.msg import Motor4Cmd,VelocityVector
from dynamic_reconfigure.server import Server
from control.cfg import ang_PIDConfig
from PID import PID_control


class JoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        # Publications
        self.pub_motor_cmd = rospy.Publisher("motor_cmd", Motor4Cmd, queue_size=1)

        # Subscriptions
        self.sub_cmd_drive = rospy.Subscriber("cmd_drive",VelocityVector,self.cbCmd,queue_size=1)
        self.sub_joy = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        self.sub_imu = rospy.Subscriber("imu/data",Imu,self.cb_imu,queue_size=1)

        #PID
        self.ang_control = PID_control("joy_stick_Angular")
        self.ang_srv = Server(ang_PIDConfig, self.ang_pid_cb, "joy_stick_Angular")
        self.ang_control.setSampleTime(0.1)
        self.ang_control.SetPoint = 0.0

        #parameter
        self.differential_constrain = rospy.get_param("differential_constrain",0.2)

        #varibles
        self.emergencyStop = False
        self.autoMode = False
        self.rotate = 0
        self.motor_msg = Motor4Cmd()
        self.last_msg = Motor4Cmd()
        self.vector_msg = VelocityVector()
        self.imu_msg = Imu()
        self.boat_angle = 0
        self.dest_angle = 0
        self.last_msg.lf = 0
        self.last_msg.lr = 0
        self.last_msg.rf = 0
        self.last_msg.rr = 0
        self.motor_stop()

        #timer
        self.timer = rospy.Timer(rospy.Duration(0.05),self.cb_publish)

    def cb_publish(self,event):
        if self.emergencyStop:
            self.motor_stop()
            self.pub_motor_cmd.publish(self.motor_msg)

        else:#low pass filter
            if not self.autoMode:
                if self.rotate > 0:
                    self.dest_angle += 5
                elif self.rotate < 0:
                    self.dest_angle -= 5
                self.dest_angle = self.angle_range(self.dest_angle)
                angular_input = self.angle_range(self.boat_angle-self.dest_angle)
                angular_output = self.control(angular_input)
                print("angular out %f" % angular_output)
                self.vector_msg.angular = angular_output
                self.motor_msg = self.vector_to_cmd(self.vector_msg)
                self.motor_msg = self.msg_constrain(self.motor_msg)
                

            self.last_msg.lf = self.low_pass_filter(self.motor_msg.lf,self.last_msg.lf)
            self.last_msg.lr = self.low_pass_filter(self.motor_msg.lr,self.last_msg.lr)
            self.last_msg.rf = self.low_pass_filter(self.motor_msg.rf,self.last_msg.rf)
            self.last_msg.rr = self.low_pass_filter(self.motor_msg.rr,self.last_msg.rr)
            self.pub_motor_cmd.publish(self.last_msg)
    
    def low_pass_filter(self,current,last):
        current = last + max(min(current-last
                            ,self.differential_constrain)
                            ,-self.differential_constrain)
        return current

    def control(self, head_angle):
		self.ang_control.update(head_angle)
		ang_output = self.ang_control.output/(-180.)
		return ang_output

    def cbCmd(self, cmd_msg):
        if not self.emergencyStop and self.autoMode:
            msg = self.vector_to_cmd(cmd_msg)
            self.motor_msg = self.msg_constrain(msg)

    def cbJoy(self, joy_msg):
        self.processButtons(joy_msg)
        if not self.emergencyStop and not self.autoMode:
            self.joy = joy_msg
            self.vector_msg = VelocityVector()
            self.vector_msg.x = self.joy.axes[0] * -1
            self.vector_msg.y = self.joy.axes[1]
            if self.joy.axes[3] > 0:
                self.rotate = 1
            elif self.joy.axes[3] < 0:
                self.rotate = -1
            else:
                self.rotate = 0
            self.vector_msg.angular = 0

    def cb_imu(self,msg):
        self.imu_msg = msg
        quat = (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quat)
        self.boat_angle = np.degrees(yaw)

    def ang_pid_cb(self, config, level):
		print("Angular: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
		Kp = float("{Kp}".format(**config))
		Ki = float("{Ki}".format(**config))
		Kd = float("{Kd}".format(**config))
		self.ang_control.setKp(Kp)
		self.ang_control.setKi(Ki)
		self.ang_control.setKd(Kd)
		return config

    def angle_range(self, angle): # limit the angle to the range of [-180, 180]
		if angle > 180:
			angle = angle - 360
			angle = self.angle_range(angle)
		elif angle < -180:
			angle = angle + 360
			angle = self.angle_range(angle)
		return angle

    def motor_stop(self):
        self.motor_msg.lf = 0
        self.motor_msg.lr = 0
        self.motor_msg.rf = 0
        self.motor_msg.rr = 0

    def vector_to_cmd(self,vec):
        motor4msg = Motor4Cmd()
        motor4msg.lf = max(min(vec.y + vec.x,1),-1)  + vec.angular
        motor4msg.lr = max(min(vec.y - vec.x,1),-1) + vec.angular
        motor4msg.rf = max(min(vec.y - vec.x,1),-1) - vec.angular
        motor4msg.rr = max(min(vec.y + vec.x,1),-1) - vec.angular
        return motor4msg

    def msg_constrain(self,msg):
        new_msg = Motor4Cmd()
        new_msg.lf = max(min(msg.lf,1),-1)
        new_msg.lr = max(min(msg.lr,1),-1)
        new_msg.rf = max(min(msg.rf,1),-1)
        new_msg.rr = max(min(msg.rr,1),-1)
        return new_msg

    def processButtons(self, joy_msg):
        # Button A
        if (joy_msg.buttons[0] == 1):
            rospy.loginfo('A button')
        
        # Y button
        elif (joy_msg.buttons[3] == 1):
            rospy.loginfo('Y button')

        # Left back button
        elif (joy_msg.buttons[4] == 1):
            rospy.loginfo('left back button')

        # Right back button
        elif (joy_msg.buttons[5] == 1):
            rospy.loginfo('right back button')

        # Back button
        elif (joy_msg.buttons[6] == 1):
            rospy.loginfo('back button')
            
        # Start button
        elif (joy_msg.buttons[7] == 1):
            self.autoMode = not self.autoMode
            if self.autoMode:
                rospy.loginfo('going auto')
            else:
                rospy.loginfo('going manual')

        # Power/middle button
        elif (joy_msg.buttons[8] == 1):
            self.emergencyStop = not self.emergencyStop
            if self.emergencyStop:
                rospy.loginfo('emergency stop activate')
                self.motor_stop()
            else:
                rospy.loginfo('emergency stop release')
        # Left joystick button
        elif (joy_msg.buttons[9] == 1):
            rospy.loginfo('left joystick button')

        else:
            some_active = sum(joy_msg.buttons) > 0
            if some_active:
                rospy.loginfo('No binding for joy_msg.buttons = %s' % str(joy_msg.buttons))

    def on_shutdown(self):
        self.motor_stop()
        self.pub_motor_cmd.publish(self.motor_msg)
        rospy.loginfo("shutting down [%s]" %(self.node_name))

if __name__ == "__main__":
    rospy.init_node("joy_mapper",anonymous=False)
    joy_mapper = JoyMapper()
    rospy.on_shutdown(joy_mapper.on_shutdown)
    rospy.spin()
