#!/usr/bin/env python
import rospy
import math

from duckiepond.msg import Heading
from duckiepond_vehicle.msg import UsvDrive

class MotorCmd(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        
        self.pub_motor_cmd = rospy.Publisher("cmd_drive",UsvDrive,queue_size=1)
        self.sub_heading = rospy.Subscriber("boat_heading",Heading,self.cbHeading,queue_size=1)

    def cbHeading(self,msg):
        mcd_msg = UsvDrive()
        speed = msg.speed*math.sin(msg.phi)
        difference = msg.speed*math.cos(msg.phi)
        mcd_msg.left = max(min(speed - difference , 1),-1)
        mcd_msg.right = max(min(speed + difference , 1),-1)

        self.pub_motor_cmd.publish(mcd_msg)

if __name__ == "__main__":
    rospy.init_node("heading_to_usv")
    commander = MotorCmd()
    rospy.spin()