#!/usr/bin/env python
import rospy
import math

from sensor_msgs.msg import Joy
from duckiepond.msg import Heading

class JoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        # Publications
        self.pub_boat_heading = rospy.Publisher("boat_heading", Heading, queue_size=1)

        # Subscriptions
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)

    def cbJoy(self, joy_msg):
        self.joy = joy_msg
        self.publishControl()
        self.processButtons(joy_msg)

    def publishControl(self):
        boat_heading_msg = Heading()
        boat_heading_msg.speed = math.sqrt((math.pow(self.joy.axes[1],2)+math.pow(self.joy.axes[3],2))/2)
        boat_heading_msg.phi = math.atan2(self.joy.axes[1],self.joy.axes[3])
        
        self.pub_boat_heading.publish(boat_heading_msg)

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
            rospy.loginfo('start button')

        # Power/middle button
        elif (joy_msg.buttons[8] == 1):
            rospy.loginfo('middle button')

        # Left joystick button
        elif (joy_msg.buttons[9] == 1):
            rospy.loginfo('left joystick button')

        else:
            some_active = sum(joy_msg.buttons) > 0
            if some_active:
                rospy.loginfo('No binding for joy_msg.buttons = %s' % str(joy_msg.buttons))


if __name__ == "__main__":
    rospy.init_node("joy_mapper",anonymous=False)
    joy_mapper = JoyMapper()
    rospy.spin()
