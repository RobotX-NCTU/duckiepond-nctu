#!/usr/bin/env python
'''
Author: Tony Hsiao                                                              
Date:2019/02/08                                                                
Last update: 2019/02/08                                                         
end to end tensorflow
'''
import tensorflow as tf
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from duckiepond.msg import MotorCmd
from cv_bridge import CvBridge
import time
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty
from duckiepond.srv import SetValue, SetValueRequest, SetValueResponse

class EndToEndTensorflow(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name)) 


        # Param
        self.sim  = rospy.get_param('~sim', True)
        self.model_path = rospy.get_param("~model")
        self.linear_speed = 1.65

        
        # Variables
        self.brdige = CvBridge()
        self.motor_cmd = None
        self.frame_counter = 0

        # Tensorflow initial
        tf.device('/device:GPU:0')
        self.sess = tf.Session()
        self.saver = tf.train.import_meta_graph(self.model_path + ".meta")
        self.saver.restore(self.sess, self.model_path)
        self.image_tensor = self.sess.graph.get_tensor_by_name('x:0')
        self.output_tensor = self.sess.graph.get_tensor_by_name('ConvNet/fc_layer_2/BiasAdd:0')
        self.srv_linear_speed = rospy.Service('~set_linear', SetValue, self.cb_srv_set_linear_speed)


        # Timer
        rospy.Timer(rospy.Duration(0.2), self.send_motor_cmd)

        # Publisher
        if self.sim:
            from duckiepond_vehicle.msg import UsvDrive
            self.pub_motion = rospy.Publisher("~cmd_drive", UsvDrive, queue_size=1)
        else:
            self.pub_motion = rospy.Publisher("~cmd_drive", MotorCmd, queue_size=1)

        # Subscriber
        self.sub_image = rospy.Subscriber("~compressed_image", CompressedImage, self.cb_image, queue_size=1)

    def image_preprocessing(self, img):
        image_final_height = 100
        image_final_width = 200

        new_img = img[img.shape[0]/3:img.shape[0]*2/3, :, :]
        # = cv2.cvtColor(new_img[:, :, :], cv2.COLOR_RGB2GRAY)
        new_img = cv2.resize( new_img, (image_final_width, image_final_height) ) 
        new_img = new_img.astype(float)
        new_img = new_img / 255
        new_img = np.reshape(new_img, (1, -1))

        return new_img


    def cb_srv_set_linear_speed(self, req):
        self.linear_speed = req.value
        rospy.loginfo("Set Linear Speed %f." %(self.linear_speed))
        return SetValueResponse()

    def cb_image(self, msg):
        if self.frame_counter <2 :
            self.frame_counter += 1
            return
        self.frame_counter = 0

        t_start = time.clock()

        img = self.brdige.compressed_imgmsg_to_cv2(msg)

        img_process = self.image_preprocessing(img)

        pred = self.sess.run(self.output_tensor, {self.image_tensor: img_process})
        self.motor_cmd = pred

        #print("Prediciont Time = ", time.clock()-t_start)

    def send_motor_cmd(self, event):
        if self.motor_cmd is not None:
            motor_msg = None
            if self.sim:
                from duckiepond_vehicle.msg import UsvDrive
                motor_msg = UsvDrive()
            else:
                motor_msg = MotorCmd()

            motor_msg.left = self.motor_cmd[0][0] * self.linear_speed 
            motor_msg.right = self.motor_cmd[0][1] * self.linear_speed 
            motor_msg.header.stamp = rospy.Time.now()
            self.pub_motion.publish(motor_msg)

            print("Motor cmd = ", motor_msg.left, motor_msg.right)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('end_to_end', anonymous=False)
    end_to_end = EndToEndTensorflow()
    rospy.on_shutdown(end_to_end.on_shutdown)
    rospy.spin()
