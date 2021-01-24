#!/usr/bin/env python

import numpy as np
#import cv2 #Must be imported before importing tensorflow, otherwise some strange error god knows why
import rospy
import base64
from detection.msg import detectionimage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class_labels = ['front','top'] #TODO: Read from label map instead
path_to_checkpoint='/home/roman/models/bricks_model/saved_model'
#path_to_checkpoint='/home/roman/models/efficientdet_d0_coco17_tpu-32/saved_model'
pipeline_cam='nvarguscamerasrc sensor_id={} ! video/x-raw(memory:NVMM), width={}, height={}, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=True'
cam_width = 800
cam_height = 600
fps = 2  # Frames per second

left_range_x = (int)(cam_width * 1/4)    # Left boundary for cm range
right_range_x = (int)(cam_width * 3/4)		# Right boundary for cm range
top_range_y = (int)(cam_height * 1/3)		# Top boundary for cm range
bottom_range_y = (int)(cam_height * 2/3)	# Lower boundary for cm range

fps_videoout = 5
fourcc_videoout = 0 # cv2.VideoWriter_fourcc(*"mp4v")
detect_threshold = 0.6

h_cm=1.9 #Height of brick


class Node:
    def callback(self,det):
        #b=s.data.encode()
        dec=base64.b64decode(det.imageBase64)
        img=np.frombuffer(dec, dtype=np.int8).reshape(det.imageShape)
        self.detection_pub.publish(self.bridge.cv2_to_imgmsg(img))

    # Power on and prepare for general usage.
    # This will activate the device and take it out of sleep mode (which must be done
    # after start-up). This function also sets both the accelerometer and the gyroscope
    # to default range settings, namely +/- 2g and +/- 250 degrees/sec.
    def __init__(self):
        rospy.init_node('translation_node')
        rospy.Subscriber("detectionimage", detectionimage, self.callback,queue_size=2) #Drop images if cannot process quick enough

        self.bridge = CvBridge()

        # Initialize the node and name it.
        self.detection_pub = rospy.Publisher("image", Image, queue_size=2)
        rospy.loginfo(rospy.get_name() + " started translation_node.")
        rospy.spin()

# Main function.
if __name__ == '__main__':
    node = Node()

