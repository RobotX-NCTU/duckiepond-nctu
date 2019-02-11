#!/usr/bin/env python
'''
Author: Tony Hsiao                                                              
Date:2019/02/11                                                                
Last update: 2019/02/11                                                      
end to end ncs
'''
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from duckiepond.msg import MotorCmd
from cv_bridge import CvBridge
import time
from mvnc import mvncapi as mvnc

class EndToEndNCS(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name)) 


        # Param
        self.sim  = rospy.get_param('~sim', True)
        self.model_path = rospy.get_param("~model")

        # Variables
        self.brdige = CvBridge()
        self.motor_cmd = None
        self.frame_counter = 0

        # NCS
        rospy.loginfo("finding NCS devices...")
        self.devices = mvnc.enumerate_devices()
        if len(self.devices) == 0:
            rospy.loginfo("No devices found. Please plug in a NCS")
        rospy.loginfo("found {} devices. "
            "opening device0...".format(len(self.devices)))
        self.device = mvnc.Device(self.devices[0])
        self.device.open()  

        with open(self.model_path, mode="rb") as f:
            self.graph_in_memory = f.read() 

        rospy.loginfo("allocating the graph on the NCS...")
        self.graph = mvnc.Graph("Imitation-Learning")
        self.fifo_in , self.fifo_out = self.graph.allocate_with_fifos(self.device, self.graph_in_memory)

        # Timer
        rospy.Timer(rospy.Duration(0.2), self.send_motor_cmd)

        # Publisher
        if self.sim:
            from duckiepond_vehicle.msg import UsvDrive
            self.pub_motion = rospy.Publisher("~cmd_drive", UsvDrive, queue_size=1)
        else:
            self.pub_motion = rospy.Publisher("~cmd_drive", MotorCmd, queue_size=1)

        # Subscriber
        self.sub_image = rospy.Subscriber("~compressed_image", CompressedImage, self.cb_image, queue_size=1, buff_size=2**24)

    def image_preprocessing(self, img):
        image_final_height = 100
        image_final_width = 200

        new_img = img[img.shape[0]/3:img.shape[0]*2/3, :, :]
        # = cv2.cvtColor(new_img[:, :, :], cv2.COLOR_RGB2GRAY)
        new_img = cv2.resize( new_img, (image_final_width, image_final_height) ) 
        new_img = new_img.astype(float)
        new_img = new_img / 255
        new_img = np.reshape(new_img, (1, -1))
        new_img = new_img.astype(np.float32)

        return new_img

    def cb_image(self, msg):
        if self.frame_counter <2 :
            self.frame_counter += 1
            return
        self.frame_counter = 0

        t_start = time.clock()

        img = self.brdige.compressed_imgmsg_to_cv2(msg)

        img_process = self.image_preprocessing(img)

        pred = self.prediction(img_process)

        self.motor_cmd = pred

        #print("Prediciont Time = ", time.clock()-t_start)

    def prediction(self, img):


        self.graph.queue_inference_with_fifo_elem(self.fifo_in, self.fifo_out, img, None)
        (output, _) = self.fifo_out.read_elem()

        return output

    def send_motor_cmd(self, event):
        if self.motor_cmd is not None:
            motor_msg = None
            if self.sim:
                from duckiepond_vehicle.msg import UsvDrive
                motor_msg = UsvDrive()
            else:
                motor_msg = MotorCmd()

            motor_msg.left = self.motor_cmd[0]
            motor_msg.right = self.motor_cmd[1]
            motor_msg.header.stamp = rospy.Time.now()
            self.pub_motion.publish(motor_msg)

            print("Motor cmd = ", motor_msg.left, motor_msg.right)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('end_to_end', anonymous=False)
    end_to_end = EndToEndNCS()
    rospy.on_shutdown(end_to_end.on_shutdown)
    rospy.spin()
