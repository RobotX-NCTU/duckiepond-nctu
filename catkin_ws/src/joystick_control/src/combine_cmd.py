#!/usr/bin/env python
import rospy
from duckiepond.msg import MotorCmd
from std_msgs.msg import Header

class Combine_moos_ssd():
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing" %self.node_name)

        self.moos_cmd = MotorCmd()
        self.ssd_cmd = MotorCmd()
        self.cmd = MotorCmd()
        self.moos_ratio = 0.5
        self.ssd_ratio = 1 - self.moos_ratio

        self.moos_time = rospy.Time.now()
        self.ssd_time = rospy.Time.now()
        
        self.sub_moos = rospy.Subscriber("moos_drive",MotorCmd,self.cb_moos,queue_size=1)
        self.sub_ssd = rospy.Subscriber("ssd_drive",MotorCmd,self.cb_ssd,queue_size=1)
        self.pub_cmd = rospy.Publisher("cmd_drive",MotorCmd,queue_size=1)
        
        self.timer = rospy.Timer(rospy.Duration(0.2),self.cb_publish)

    def cb_moos(self,msg):
        self.moos_time = rospy.Time.now()
        self.moos_cmd = msg

    def cb_ssd(self,msg):
        self.ssd_time = rospy.Time.now()
        self.ssd_cmd = msg
    
    def cb_publish(self,event):
        time_now = rospy.Time.now()
        moos_diff = time_now.to_sec() - self.moos_time.to_sec()
        ssd_diff = time_now.to_sec() - self.ssd_time.to_sec()

        if moos_diff<1.5 and  ssd_diff<1.5:
            self.cmd.right = self.moos_cmd.right * self.moos_ratio
            self.cmd.left = self.moos_cmd.left * self.moos_ratio
            self.cmd.right += self.ssd_cmd.right * self.ssd_ratio
            self.cmd.left += self.ssd_cmd.left * self.ssd_ratio
        elif moos_diff>1.5 and ssd_diff<1.5:
            self.cmd = self.ssd_cmd
        elif moos_diff<1.5 and ssd_diff>1.5:
            self.cmd = self.moos_cmd
        else:
            self.cmd.right = 0
            self.cmd.left = 0
        self.pub_cmd.publish(self.cmd)
        

    def on_shutdown(self):
        self.cmd.right = 0
        self.cmd.left = 0
        self.pub_cmd.publish(self.cmd)


if __name__ == "__main__":
    rospy.init_node("combine")
    combine = Combine_moos_ssd()
    rospy.on_shutdown(combine.on_shutdown)
    rospy.spin()
