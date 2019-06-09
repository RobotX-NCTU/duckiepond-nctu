#!/usr/bin/env python3

import numpy as np
import os.path
import tensorflow as tf
import cv2
import rospy
import sys
from centroidtracker import CentroidTracker
from duckiepond.msg import Box, Boxlist
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError



class ObjectDetecter(object):
    def __init__(self):
        self.target_name = rospy.get_param("detecter/target","boat")
        my_dir = os.path.abspath(os.path.dirname(__file__))
        self.PATH_TO_CKPT = os.path.join(
            my_dir, "../graphs/"+self.target_name+"_graph.pb")
        self.NUM_CLASSES = 1
        self.session_config = tf.ConfigProto()
        self.session_config.gpu_options.allow_growth = True
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        self.sess = tf.Session(graph=self.detection_graph,
                               config=self.session_config)
        #self.tracker = CentroidTracker()
        # parameter
        self.publish_image = rospy.get_param("detecter/publish_image", True)
        self.dest_rate = rospy.get_param("detecter/dest_rate", 30)
        self.input_rate = rospy.get_param("detecter/input_rate", 30)
        self.threshold = rospy.get_param("detecter/threshold", 0.2)
        self.sim = rospy.get_param("detecter/sim", False)

        self.node_name = rospy.get_name()
        if self.sim:
            self.subscriber = rospy.Subscriber(
                "camera/rgb/image_rect_color", Image, self.cbimage_raw, queue_size=1, buff_size=2**24)
        else:
            self.subscriber = rospy.Subscriber(
                "camera_node/image/compressed", CompressedImage, self.cbimage, queue_size=1, buff_size=2**24)

        self.pubImg = rospy.Publisher(
            "detecter/image/compressed", CompressedImage, queue_size=1)
        self.pubBoxlist = rospy.Publisher(
            "detecter/predictions", Boxlist, queue_size=1)

        rospy.loginfo("[%s] Initializing " % (self.node_name))
        self.bridge = CvBridge()
        self.frame_counter = 0
        self.image_np = np.zeros((800, 600, 3), np.uint8)

    def cbimage_raw(self, img):
        self.frame_counter += 1
        if self.frame_counter == self.input_rate/self.dest_rate:
            self.frame_counter = 0
            image = self.bridge.imgmsg_to_cv2(img)
            self.process_img(
                image, self.bridge.cv2_to_compressed_imgmsg(image))

    def cbimage(self, img):
        self.frame_counter += 1
        if self.frame_counter == self.input_rate/self.dest_rate:
            self.frame_counter = 0
            image = self.bridge.compressed_imgmsg_to_cv2(img)
            self.process_img(image, img)

    def process_img(self, image, msg):
        image_for_result = image
        box_list = Boxlist()
        (box_list.image_height, box_list.image_width, _) = image.shape
        box_list.image = msg
        predictions, boxes = self.predict(image, self.sess)
        #objects = self.tracker.update(boxes)

        for pred in predictions:
            (pred_class, pred_conf, ptA, ptB) = pred

            if pred_class==1 and pred_conf > self.threshold:
                box = Box()
                box.x = ptA[0]
                box.y = ptA[1]
                box.w = ptB[0] - ptA[0]
                box.h = ptB[1] - ptA[1]
                #box.id = self.tracker.search_id(ptA,ptB)
                box.id = 0
                box.confidence = pred_conf
                box_list.list.append(box)

                if self.publish_image:
                    label = "id:{},{:.2f}%".format(box.id,
                                                 pred_conf * 100)
                    (startX, startY) = (ptA[0], ptA[1])
                    y = startY - 15 if startY - 15 > 15 else startY + 15

                    cv2.rectangle(image_for_result, ptA, ptB,
                                  (255,255,0), 2)
                    cv2.rectangle(image_for_result, (ptA[0],ptA[1]-20), (ptA[0]+130,ptA[1]),
                                  (255,255,0), -1)
                    cv2.putText(image_for_result, label, (ptA[0]+5,ptA[1]-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)

        if self.publish_image:
            img_msg = self.bridge.cv2_to_compressed_imgmsg(
                image_for_result)
            img_msg.header = Header()
            img_msg.header.stamp = rospy.Time.now()
            self.pubImg.publish(img_msg)
        self.pubBoxlist.publish(box_list)

    def predict(self, image, sess):
        image_np_expanded = np.expand_dims(image, axis=0)
        image_tensor = self.detection_graph.get_tensor_by_name(
            'image_tensor:0')
        boxes = self.detection_graph.get_tensor_by_name(
            'detection_boxes:0')
        scores = self.detection_graph.get_tensor_by_name(
            'detection_scores:0')
        classes = self.detection_graph.get_tensor_by_name(
            'detection_classes:0')
        num_detections = self.detection_graph.get_tensor_by_name(
            'num_detections:0')
        (boxes, scores, classes, num_detections) = self.sess.run(
            [boxes, scores, classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})

        conf_list = np.squeeze(scores)
        class_list = np.squeeze(classes).astype(np.int32)
        box_list = np.squeeze(boxes)

        predictions = []
        boxes = []
        num = int(num_detections[0])
        for i in range(num):
            pred_boxpts = box_list[i]
            ptA = (int(pred_boxpts[1]*image.shape[1]),
                   int(pred_boxpts[0]*image.shape[0]))
            ptB = (int(pred_boxpts[3]*image.shape[1]),
                   int(pred_boxpts[2]*image.shape[0]))
            boxes.append((ptA[0], ptA[1], ptB[0], ptB[1]))
            predictions.append((class_list[i], conf_list[i], ptA, ptB))

        return predictions, boxes

    def on_shutdown(self):
        self.sess.close()
        print ('shutting down')


if __name__ == "__main__":
    rospy.init_node("detecter")
    node = ObjectDetecter()
    rospy.on_shutdown(node.on_shutdown)
    rospy.spin()
