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
from csi_camera import *

class_labels = ['front','top'] #TODO: Read from label map instead
path_to_checkpoint='/home/roman/models/bricks_model/saved_model'

cam_width = 800
cam_height = 600
cam_fps = 15 #fps to be used in camera pipeline
detect_fps = 5  # detect frames per second (this is approximately as much as we can get, anyway)
stream_fps = 3  # image frames per second (do this less frequently, to save bandwith in ros)

img_resize = 0.5 #Resize image, to reduce network traffic (RViz can't see very well anyway...)

left_range_x = (int)(cam_width * 1/4)    # Left boundary for cm range
right_range_x = (int)(cam_width * 3/4)		# Right boundary for cm range
top_range_y = (int)(cam_height * 1/3)		# Top boundary for cm range
bottom_range_y = (int)(cam_height * 2/3)	# Lower boundary for cm range

detect_threshold = 0.6


class Node:
    def callback(self,timer_event):
        rospy.loginfo(rospy.get_name() + " capturing image.")
        current_time = rospy.Time.now()
        grabbed, img0 = self.cameras.read_camera(0)
        grabbed, img1 = self.cameras.read_camera(1)
        if img0 is not None and img1 is not None:
            rospy.loginfo(rospy.get_name() + " images captured. Running detection")
            rospy.loginfo(rospy.get_name() + " image shapes are {}, {}".format(str(np.shape(img0)),str(np.shape(img1))))
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

    def shutdownhook(self):
        rospy.loginfo(rospy.get_name() + " shutdown hook called- cleaning up detection_node.")
        rospy.loginfo(rospy.get_name() + " stopping the cameras")
        self.cameras.stop_cameras()
        rospy.loginfo(rospy.get_name() + " cleanup done.")

    # Power on and prepare for general usage.
    # This will activate the device and take it out of sleep mode (which must be done
    # after start-up). This function also sets both the accelerometer and the gyroscope
    # to default range settings, namely +/- 2g and +/- 250 degrees/sec.
    def __init__(self):
        rospy.init_node('detection_node')
        self.cameras=CSI_Cameras(sensor_mode=3,height=cam_height,width=cam_width,framerate=cam_fps,flips=[2,2])
        rospy.on_shutdown(self.shutdownhook)
        if self.cameras.start_cameras():
            rospy.loginfo(rospy.get_name() + " Opened cameras")
        else:
            return
        self.last_image_sent = rospy.Time.now()
        # Open camera streamers
        rospy.loginfo(rospy.get_name() + " Opened cameras")

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




# Main function.
if __name__ == '__main__':
    node = Node()

