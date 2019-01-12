#!/usr/bin/env python
import rospy
import math

from duckiepond.msg import Motor_cmd,Heading

class MotorCmd(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        
        self.pub_motor_cmd = rospy.Publisher("motor_cmd",Motor_cmd,queue_size=1)
        self.sub_heading = rospy.Subscriber("boat_heading",Heading,self.cbHeading,queue_size=1)

    def cbHeading(self,msg):
        mcd_msg = Motor_cmd()
        speed = msg.speed*math.sin(msg.phi)
        difference = msg.speed*math.cos(msg.phi)
        mcd_msg.left = max(min(speed - difference , 1),-1)
        mcd_msg.right = max(min(speed + difference , 1),-1)

        self.pub_motor_cmd.publish(mcd_msg)

if __name__ == "__main__":
    rospy.init_node("heading_to_motor_cmd")
    commander = MotorCmd()
    rospy.spin()