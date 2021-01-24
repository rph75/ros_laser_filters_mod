#!/usr/bin/env python3

import numpy as np
import cv2 #Must be imported before importing tensorflow, otherwise some strange error god knows why
import tensorflow
import rospy
import coordinates
#import tf as transform
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from detection.msg import detection,detectionimage
import base64

class_labels = ['front','top'] #TODO: Read from label map instead
path_to_checkpoint='/home/roman/models/bricks_model/saved_model'
#path_to_checkpoint='/home/roman/models/efficientdet_d0_coco17_tpu-32/saved_model'
pipeline_cam='nvarguscamerasrc sensor_id={} ! video/x-raw(memory:NVMM), width={}, height={}, format=(string)NV12, framerate=(fraction){}/1 ! nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=True'
#pipeline_videoout='appsrc ! videoconvert ! x264enc key-int-max=12 byte-stream=true tune="zerolatency" ! mpegtsmux ! tcpserversink host=0.0.0.0 port=5000 sync=false'
pipeline_videoout='appsrc ! videoconvert ! x264enc key-int-max=12 byte-stream=true ! mpegtsmux ! filesink location=/home/roman/test.mts '
#pipeline_videoout='appsrc ! autovideoconvert ! omxh265enc ! matroskamux ! filesink location=/home/roman/test.mkv '
cam_width = 800
cam_height = 600
detect_fps = 10  # detect frames per second
stream_fps = 3  # image frames per second (do this less frequently, to save bandwith in ros)

img_resize = 0.5 #Resize image, to reduce network traffic (RViz can't see very well anyway...)

left_range_x = (int)(cam_width * 1/4)    # Left boundary for cm range
right_range_x = (int)(cam_width * 3/4)		# Right boundary for cm range
top_range_y = (int)(cam_height * 1/3)		# Top boundary for cm range
bottom_range_y = (int)(cam_height * 2/3)	# Lower boundary for cm range

fourcc_videoout = 0 # cv2.VideoWriter_fourcc(*"mp4v")
detect_threshold = 0.6

h_cm=1.9 #Height of brick


class Node:
    def callback(self,timer_event):
        current_time = rospy.Time.now()
        if self.cap0 is not None and self.cap1 is not None and self.cap0.isOpened() and self.cap1.isOpened():
            rospy.loginfo(rospy.get_name() + " capturing image.")
            ret0, img0 = self.cap0.read()
            if not ret0:
                return
            ret1, img1 = self.cap1.read()
            if not ret1:
                return
            rospy.loginfo(rospy.get_name() + " image captured. Running detection")
            #Detect far camera
            detections1 = self.detect_from_image(False, img1)
            #Detect near camera
            detections0 = self.detect_from_image(True, img0)
            rospy.loginfo(rospy.get_name() + " completed image detection")
            # publish the message
            p0,c0=self.to_points_and_colors(detections0,img0)
            p1,c1=self.to_points_and_colors(detections1,img1)
            colors = c0+c1
            points = p0+p1
            det = detection(detections=points,colors=colors)
            det.header.stamp=current_time
            det.header.frame_id="base_link"
            rospy.loginfo(rospy.get_name() + " publishing detection")
            self.detection_pub.publish(det)
            rospy.loginfo(rospy.get_name() + " detection published")
            if current_time>self.last_image_sent+rospy.Duration.from_sec(1.0/stream_fps):
                self.last_image_sent=current_time
                # Stream an image (for vizualization only)
                img1 = self.display_detections(img1, detections1)
                img0 = self.display_detections(img0, detections0)
                color = (140, 140, 140)
                cv2.line(img0, (0, top_range_y), (cam_width, top_range_y), color, 5)
                cv2.line(img0, (0, bottom_range_y), (cam_width, bottom_range_y), color, 5)
                cv2.line(img0, (left_range_x, 0), (left_range_x, cam_height), color, 5)
                cv2.line(img0, (right_range_x, 0), (right_range_x, cam_height), color, 5)

                img = cv2.vconcat([img1, img0])
                img = cv2.resize(img, (0, 0), fx=img_resize, fy=img_resize)
                #fps = round(1000. / elapsed_time, 1)
                #fps_txt = str(fps) + " FPS"
                #cv2.putText(img, fps_txt, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
                #power_brain = pc.getPower_W(1)
                #power_total = pc.getPower_W(3)
                #voltage = pc.getBusVoltage_V(3)
                #power_txt = "Brain {0:2.1f} of {1:2.1f}W".format(power_brain, power_total)
                #cv2.putText(img, power_txt, (25, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
                #voltage_txt = "Batt {0:2.2f} V".format(voltage)
                #if voltage > critical_voltage:
                #    cv2.putText(img, voltage_txt, (25, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
                #else:
                #    cv2.putText(img, voltage_txt, (25, 100), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 3)

                #cv2.imshow('Detection', img)
                detimage = detectionimage()
                detimage.imageShape=list(img.shape)
                detimage.imageBase64= base64.b64encode(img.tobytes()).decode()
                self.detectionimage_pub.publish(detimage)

    def to_points_and_colors(self,detections,img):
        points = []
        colors = []
        for d in detections:
            x_cm=d["x_cm"]
            y_cm=d["y_cm"]
            if x_cm is not None and y_cm is not None:
                p = coordinates.to_base_link_point(x_cm,y_cm)
                points.append(p)
                c = np.array(cv2.mean(img[d["y_min_pix"]:d["y_max_pix"], d["x_min_pix"]: d["x_max_pix"]])) / 255
                colors.append(ColorRGBA(b=c[0], g=c[1], r=c[2], a=1.0))  # Array is BGR
        return points,colors

    def detect_from_image(self,is_near, img):
        im_height, im_width, _ = img.shape
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        input_tensor = np.expand_dims(img, 0)
        det = self.detect_fn(input_tensor)

        bboxes = det['detection_boxes'][0].numpy()
        bclasses = det['detection_classes'][0].numpy().astype(np.int32)
        bscores = det['detection_scores'][0].numpy()
        detections = self.extract_detections(is_near, bboxes, bclasses, bscores, im_width, im_height)

        return detections

    #Returns array of Detection:
    # x_min (pixels)
    # y_min (pixels)
    # x_max (pixels)
    # y_max (pixels)
    # label (string)
    # score (double 0-1)
    # x_cm (cm or None) of bottom left corner
    # y_cm (cm or None) of bottom left corner

    def extract_detections(self,is_near, bboxes, bclasses, bscores, im_width, im_height):
        detections = []
        for idx in range(len(bboxes)):
            if bscores[idx] >= detect_threshold:
                y_min_pix = int(bboxes[idx][0] * im_height)
                x_min_pix = int(bboxes[idx][1] * im_width)
                y_max_pix = int(bboxes[idx][2] * im_height)
                x_max_pix = int(bboxes[idx][3] * im_width)

                if is_near:
                    x_cm, y_cm = coordinates.near_pixel_to_cm(x_min_pix, y_max_pix)  # Bottom left corner
                else:
                    x_cm, y_cm = coordinates.far_pixel_to_cm(x_min_pix, y_max_pix)

                label = class_labels[int(bclasses[idx])-1]
                score=float(bscores[idx])
                #bbox.append([x_min, y_min, x_max, y_max, label, score,x_cm, y_cm ])
                d = {"x_min_pix": x_min_pix, "y_min_pix": y_min_pix, "y_max_pix": y_max_pix, "x_max_pix": x_max_pix,
                     "label": label, "score": score, "x_cm": x_cm, "y_cm": y_cm}
                detections.append(d)
        return detections

    def display_detections(self,image, detections):
        img = image.copy()
        for idx in range(len(detections)):
            d = detections[idx]
            text = "{0}: {1:0.1f}%, {2:1d} x {3:1d} pix".format(d["label"], d["score"] * 100, d["x_max_pix"] - d["x_min_pix"],
                                                                d["y_max_pix"] - d["y_min_pix"])
            coord_pix = "x = {0:1d}, y = {1:1d} pix".format(d["x_min_pix"], d["y_max_pix"])
            if d["x_cm"] is not None and d["y_cm"] is not None:
                coord_cm = "x = {0:.1f}, y = {1:.1f} cm".format(d["x_cm"], d["y_cm"])
            else:
                coord_cm = "N/A"

            color = (0, 255, 0)
            cv2.rectangle(img, (d["x_min_pix"], d["y_min_pix"]), (d["x_max_pix"], d["y_max_pix"]), color, 4)
            cv2.rectangle(img, (d["x_min_pix"], d["y_min_pix"] - 37), (d["x_min_pix"], d["y_min_pix"]), (255, 255, 255), 4)
            cv2.putText(img, text, (d["x_min_pix"] + 5, d["y_min_pix"] - 27), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (127, 255, 0), 1)
            cv2.putText(img, coord_pix, (d["x_min_pix"] + 5, d["y_min_pix"] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0),
                        1)
            cv2.putText(img, coord_cm, (d["x_min_pix"] + 5, d["y_min_pix"] - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0),
                        1)
        return img

    # Power on and prepare for general usage.
    # This will activate the device and take it out of sleep mode (which must be done
    # after start-up). This function also sets both the accelerometer and the gyroscope
    # to default range settings, namely +/- 2g and +/- 250 degrees/sec.
    def __init__(self):
        rospy.init_node('detection_node')
        self.cap0 = None
        self.cap1 = None
        self.last_image_sent = rospy.Time.now()
        # Open camera streamers
        self.cap0 = cv2.VideoCapture(pipeline_cam.format(0, cam_width, cam_height, detect_fps))
        rospy.loginfo(rospy.get_name() + " Opened camera 0")
        self.cap1 = cv2.VideoCapture(pipeline_cam.format(1, cam_width, cam_height, detect_fps))
        rospy.loginfo(rospy.get_name() + " Opened camera 1")

        # Load model
        rospy.loginfo(rospy.get_name() + " Loading model...")
        tensorflow.keras.backend.clear_session()
        start = rospy.Time.now()
        self.detect_fn = tensorflow.saved_model.load(path_to_checkpoint)
        rospy.loginfo(rospy.get_name() + " ...model loaded in {} seconds".format((rospy.Time.now()-start)/1000000000))

        # Initialize the node and name it.
        self.detection_pub = rospy.Publisher("detection", detection, queue_size=10)
        self.detectionimage_pub = rospy.Publisher("detectionimage", detectionimage, queue_size=2)
        interval = rospy.Duration.from_sec(1.0/detect_fps)
        rospy.Timer(interval, self.callback)
        rospy.loginfo(rospy.get_name() + " started detection_node.")
        rospy.spin()

    # Deleting (Calling destructor)
    def __del__(self):
        rospy.loginfo(rospy.get_name() + " cleaning up detection_node.")
        if self.cap0 is not None:
            self.cap0.release()
        if self.cap1 is not None:
            self.cap1.release()


# Main function.
if __name__ == '__main__':
    node = Node()

