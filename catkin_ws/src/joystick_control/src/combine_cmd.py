#!/usr/bin/env python
import rospy
from duckiepond.msg import MotorCmd
from std_msgs.msg import Header,Float64
import math

class Combine_moos_ssd():
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing" %self.node_name)

        self.moos_cmd = MotorCmd()
        self.ssd_cmd = MotorCmd()
        self.cmd = MotorCmd()
        
        self.max_d = 16

        self.moos_time = rospy.Time.now()
        self.ssd_time = rospy.Time.now()

        self.sub_bx = rospy.Subscriber("/BRIAN/NAV_X",Float64,self.cb_bx,queue_size=1)
        self.sub_by = rospy.Subscriber("/BRIAN/NAV_Y",Float64,self.cb_by,queue_size=1)
        self.sub_mx = rospy.Subscriber("/MONICA/NAV_X",Float64,self.cb_mx,queue_size=1)
        self.sub_my = rospy.Subscriber("/MONICA/NAV_Y",Float64,self.cb_my,queue_size=1)
        self.bx = 0
        self.by = 0
        self.mx = 0
        self.my = 0

        self.sub_moos = rospy.Subscriber("moos_drive",MotorCmd,self.cb_moos,queue_size=1)
        self.sub_ssd = rospy.Subscriber("ssd_drive",MotorCmd,self.cb_ssd,queue_size=1)
        self.pub_cmd = rospy.Publisher("cmd_drive",MotorCmd,queue_size=1)
        
        self.timer = rospy.Timer(rospy.Duration(0.2),self.cb_publish)
    
    def cb_bx(self,msg):
        self.bx = msg.data
    
    def cb_by(self,msg):
        self.by = msg.data
    
    def cb_mx(self,msg):
        self.mx = msg.data
    
    def cb_my(self,msg):
        self.my = msg.data

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
            dis = pow((self.bx-self.mx),2) + pow((self.by-self.my),2)
            dis = pow(dis,0.5)
            if dis > self.max_d:
                dis = self.max_d
            moos_ratio = dis/float(self.max_d)
            ssd_ratio = 1 - moos_ratio
            self.cmd.right = self.moos_cmd.right * moos_ratio
            self.cmd.left = self.moos_cmd.left * moos_ratio
            self.cmd.right += self.ssd_cmd.right * ssd_ratio
            self.cmd.left += self.ssd_cmd.left * ssd_ratio
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
