#!/usr/bin/env python
import rospy
import math

from sensor_msgs.msg import Joy
from duckiepond.msg import Motor4Cmd,VelocityVector

class JoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        # Publications
        self.pub_motor_cmd = rospy.Publisher("motor_cmd", Motor4Cmd, queue_size=1)

        # Subscriptions
        self.sub_cmd_drive = rospy.Subscriber("cmd_drive",VelocityVector,self.cbCmd,queue_size=1)
        self.sub_joy = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)

        #varibles
        self.emergencyStop = False
        self.autoMode = False
        self.motor_msg = Motor4Cmd()
        self.motor_stop()

        #timer
        self.timer = rospy.Timer(rospy.Duration(0.2),self.cb_publish)

    def cb_publish(self,event):
        if self.emergencyStop:
            self.motor_stop()
        
        self.pub_motor_cmd.publish(self.motor_msg)

    def cbCmd(self, cmd_msg):
        if not self.emergencyStop and self.autoMode:
            msg = self.vector_to_cmd(cmd_msg)
            self.motor_msg = self.msg_constrain(msg)

    def cbJoy(self, joy_msg):
        self.processButtons(joy_msg)
        if not self.emergencyStop and not self.autoMode:
            self.joy = joy_msg
            vector_msg = VelocityVector()
            vector_msg.x = self.joy.axes[0]
            vector_msg.y = self.joy.axes[1] * -1
            vector_msg.angular = self.joy.axes[3] * -1

            self.motor_msg = self.vector_to_cmd(vector_msg)
            self.motor_msg = self.msg_constrain(self.motor_msg)

    def motor_stop(self):
        self.motor_msg.lf = 0
        self.motor_msg.lr = 0
        self.motor_msg.rf = 0
        self.motor_msg.rr = 0

    def vector_to_cmd(self,vec):
        motor4msg = Motor4Cmd()
        motor4msg.lf = vec.y + vec.x + vec.angular
        motor4msg.lr = vec.y - vec.x + vec.angular
        motor4msg.rf = vec.y - vec.x - vec.angular
        motor4msg.rr = vec.y + vec.x - vec.angular
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
