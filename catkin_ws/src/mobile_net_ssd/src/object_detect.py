#!/usr/bin/env python

from mvnc import mvncapi as mvnc
import numpy as np
import rospy
import time
import cv2
import os.path
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge, CvBridgeError

CLASSES = ("background", "aeroplan", "bicycle", "bird",
			"boat", "bottle", "bus", "car", "cat", "chair", "cow",
			"diningtable", "dog", "horse", "motorbike", "person",
			"pottedplant", "sheep", "sofa", "train", "tvmonitor")
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

class ObjectDetecter(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		self.subscriber = rospy.Subscriber("camera_node/image/compressed",CompressedImage,self.cbimage,queue_size=1)
		self.pubimage = rospy.Publisher("detecter/image/compressed",CompressedImage,queue_size=1)
		rospy.loginfo("[%s] Initializing " %(self.node_name))
		self.bridge = CvBridge()

		self.PREPROCESS_DIMS = (300,300)
		self.DISPLAY_DIMS = (900,900)

		# calculate the multiplier needed to scale the bounding boxes
		self.DISP_MULTIPLIER = self.DISPLAY_DIMS[0] // self.PREPROCESS_DIMS[0]

		# grab a list of all NCS devices plugged in to USB
		print("[INFO] finding NCS devices...")
		self.devices = mvnc.enumerate_devices()

		# if no devices found, exit the script
		if len(self.devices) == 0:
			print("[INFO] No devices found. Please plug in a NCS")
		# use the first device since this is a simple test script
		# (you'll want to modify this is using multiple NCS devices)
		print("[INFO] found {} devices. device0 will be used. "
			"opening device0...".format(len(self.devices)))
		self.device = mvnc.Device(self.devices[0])
		self.device.open()

		# open the CNN graph file
		print("[INFO] loading the graph file into memory...")
		my_dir = os.path.abspath(os.path.dirname(__file__))
		path = os.path.join(my_dir, "../graphs/mobilenetgraph")
		with open(path, mode="rb") as f:
			graph_in_memory = f.read()

		# load the graph into the NCS
		print("[INFO] allocating the graph on the NCS...")
		self.graph = mvnc.Graph("MobileNet-SSD")
		self.ssd_fifo_in , self.ssd_fifo_out = self.graph.allocate_with_fifos(self.device,graph_in_memory)

	def cbimage(self,img):
		# grab the frame from the threaded video stream
		# make a copy of the frame and resize it for display/video purposes
		image = self.bridge.compressed_imgmsg_to_cv2(img)
		image_for_result = cv2.resize(image, self.DISPLAY_DIMS)

		# use the NCS to acquire predictions
		predictions = self.predict(image)

		# loop over our predictions
		for (i, pred) in enumerate(predictions):
			# extract prediction data for readability
			(pred_class, pred_conf, pred_boxpts) = pred

			# filter out weak detections by ensuring the `confidence`
			# is greater than the minimum confidence
			if pred_class==4:
				# print prediction to terminal
				print("[INFO] Prediction #{}: class={}, confidence={}, "
					"boxpoints={}".format(i, CLASSES[pred_class], pred_conf,
					pred_boxpts))

				# build a label consisting of the predicted class and
				# associated probability
				label = "{}: {:.2f}%".format(CLASSES[pred_class],
					pred_conf * 100)

				# extract information from the prediction boxpoints
				(ptA, ptB) = (pred_boxpts[0], pred_boxpts[1])
				ptA = (ptA[0] * self.DISP_MULTIPLIER, ptA[1] * self.DISP_MULTIPLIER)
				ptB = (ptB[0] * self.DISP_MULTIPLIER, ptB[1] * self.DISP_MULTIPLIER)
				(startX, startY) = (ptA[0], ptA[1])
				y = startY - 15 if startY - 15 > 15 else startY + 15

				# display the rectangle and label text
				cv2.rectangle(image_for_result, ptA, ptB,
					COLORS[pred_class], 2)
				cv2.putText(image_for_result, label, (startX, y),
					cv2.FONT_HERSHEY_SIMPLEX, 1, COLORS[pred_class], 3)
			
		self.pubimage.publish(self.bridge.cv2_to_compressed_imgmsg(image_for_result))


	def preprocess_image(self,input_image):
		# preprocess the image
		preprocessed = cv2.resize(input_image, self.PREPROCESS_DIMS)
		preprocessed = preprocessed - 127.5
		preprocessed = preprocessed * 0.007843
		preprocessed = preprocessed.astype(np.float32)

		# return the image to the calling function
		return preprocessed

	def predict(self,image):
		# preprocess the image
		image = self.preprocess_image(image)

		# send the image to the NCS and run a forward pass to grab the
		# network predictions
		self.graph.queue_inference_with_fifo_elem(self.ssd_fifo_in,self.ssd_fifo_out,image,None)
		(output, _) = self.ssd_fifo_out.read_elem()

		# grab the number of valid object predictions from the output,
		# then initialize the list of predictions
		num_valid_boxes = output[0]
		predictions = []

		#loop over results
		for box_index in range(num_valid_boxes):
			# calculate the base index into our array so we can extract
			# bounding box information
			base_index = 7 + box_index * 7

			# boxes with non-finite (inf, nan, etc) numbers must be ignored
			if (not np.isfinite(output[base_index]) or
				not np.isfinite(output[base_index + 1]) or
				not np.isfinite(output[base_index + 2]) or
				not np.isfinite(output[base_index + 3]) or
				not np.isfinite(output[base_index + 4]) or
				not np.isfinite(output[base_index + 5]) or
				not np.isfinite(output[base_index + 6])):
				continue

			# extract the image width and height and clip the boxes to the
			# image size in case network returns boxes outside of the image
			# boundaries
			(h, w) = image.shape[:2]
			x1 = max(0, int(output[base_index + 3] * w))
			y1 = max(0, int(output[base_index + 4] * h))
			x2 = min(w,	int(output[base_index + 5] * w))
			y2 = min(h,	int(output[base_index + 6] * h))

			# grab the prediction class label, confidence (i.e., probability),
			# and bounding box (x, y)-coordinates
			pred_class = int(output[base_index + 1])
			pred_conf = output[base_index + 2]
			pred_boxpts = ((x1, y1), (x2, y2))

			# create prediciton tuple and append the prediction to the
			# predictions list
			prediction = (pred_class, pred_conf, pred_boxpts)
			predictions.append(prediction)

		# return the list of predictions to the calling function
		return predictions



	def on_shutdown(self):
		# clean up the graph and device
		self.ssd_fifo_in.destroy()
		self.ssd_fifo_out.destroy()
		self.graph.destroy()
		self.device.close()
		self.device.destroy()


if __name__ == "__main__":
	rospy.init_node("detecter")
	node = ObjectDetecter()
	rospy.on_shutdown(node.on_shutdown)
	rospy.spin()