#!/usr/bin/env python

import numpy as np
import os.path
import tensorflow as tf
import cv2
import rospy

from PIL import Image
from utils import label_map_util
from utils import visualization_utils as vis_util

from std_msgs.msg import Header
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class ObjectDetecter(object):
    def __init__(self):
        my_dir = os.path.abspath(os.path.dirname(__file__))
        self.PATH_TO_CKPT = os.path.join(my_dir, "../graphs/frozen_inference_graph.pb")
        self.PATH_TO_LABELS = os.path.join(my_dir, "../annotations/label_map.pbtxt")
        self.NUM_CLASSES = 1

        self.detection_graph = tf.Graph()
        self.session_config = tf.ConfigProto()
        self.session_config.gpu_options.allow_growth = True
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        # Loading label map
        # Label maps map indices to category names, so that when our convolution network predicts `5`, we know that this corresponds to `airplane`.  Here we use internal utility functions, but anything that returns a dictionary mapping integers to appropriate string labels would be fine
        self.label_map = label_map_util.load_labelmap(self.PATH_TO_LABELS)
        self.categories = label_map_util.convert_label_map_to_categories(
            self.label_map, max_num_classes=self.NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(self.categories)

		#parameter
        self.publish_image = rospy.get_param("detecter/publish_image",True)
        self.dest_rate = rospy.get_param("detecter/dest_rate",30)
        self.input_rate = rospy.get_param("detecter/input_rate",30)
        self.threshold = rospy.get_param("detecter/threshold",0.2)

        self.node_name = rospy.get_name()
        self.subscriber = rospy.Subscriber("/alpha/camera_node/image/compressed",CompressedImage,self.cbimage,queue_size=1,buff_size=2**24)
        self.pubImg = rospy.Publisher("detecter/image/compressed",CompressedImage,queue_size=1)

        rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.bridge = CvBridge()
        self.frame_counter = 0


    def cbimage(self,img):
        self.frame_counter += 1
        if self.frame_counter == self.input_rate/self.dest_rate:
            self.frame_counter=0

            image_np = self.bridge.compressed_imgmsg_to_cv2(img)

            # Detection
            with self.detection_graph.as_default():
                with tf.Session(graph=self.detection_graph,config=self.session_config) as sess:

                    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                    image_np_expanded = np.expand_dims(image_np, axis=0)
                    # Extract image tensor
                    image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                    # Extract detection boxes
                    boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                    # Extract detection scores
                    scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                    # Extract detection classes
                    classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                    # Extract number of detectionsd
                    num_detections = self.detection_graph.get_tensor_by_name(
                        'num_detections:0')
                    # Actual detection.
                    (boxes, scores, classes, num_detections) = sess.run(
                        [boxes, scores, classes, num_detections],
                        feed_dict={image_tensor: image_np_expanded})
            
            # Visualization of the results of a detection.
            vis_util.visualize_boxes_and_labels_on_image_array(
                image_np,
                np.squeeze(boxes),
                np.squeeze(classes).astype(np.int32),
                np.squeeze(scores),
                self.category_index,
                use_normalized_coordinates=True,
                line_thickness=8)

            if self.publish_image:
				img_msg = self.bridge.cv2_to_compressed_imgmsg(image_np)
				img_msg.header = Header()
				img_msg.header.stamp = rospy.Time.now()
				self.pubImg.publish(img_msg)

    def on_shutdown(self):
		pass


if __name__ == "__main__":
	rospy.init_node("detecter")
	node = ObjectDetecter()
	rospy.on_shutdown(node.on_shutdown)
	rospy.spin()
	
