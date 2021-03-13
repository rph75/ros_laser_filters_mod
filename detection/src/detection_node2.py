#!/usr/bin/env python3

from std_srvs.srv import Empty
import numpy as np
import cv2 #Must be imported before importing tensorflow, otherwise some strange error god knows why
import tensorflow
import rospy
from camera_coord import CameraCoordinates
import threading
#import tf as transform
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from detection.msg import singledetection ,detectionarray,detectionimage
import base64
from rospy import Duration
class_labels = ['front','top'] #TODO: Read from label map instead
path_to_checkpoint='/home/roman/models/bricks_model/saved_model'

cam_width = 800
cam_height = 600
cam_fps = 15    #fps to be used in camera pipeline
detect_fps = 10  # detect frames per second
stream_fps = 3  # image frames per second (doing this less frequently saves bandwith in ros)
img_resize = 0.2 #Resize image, to reduce network traffic (RViz can't see very well anyway...)

detect_threshold = 0.6
OVERLAP_THRESHOLD = 0.8 #If area overlaps more than this value, consider this a dual detection / overlap

debug_detect = True  # For debug: Do detection or camera only
debug_show = False

class Detection:
    def __init__(self,time, source, x_min_pix, y_min_pix, x_max_pix, y_max_pix, label, score, x_m, y_m, width_m, is_valid):
        self.time = time
        self.x_min_pix = x_min_pix
        self.y_min_pix = y_min_pix
        self.y_max_pix = y_max_pix
        self.x_max_pix = x_max_pix
        self.label = label
        self.score = score
        self.x_m = x_m
        self.y_m = y_m
        self.width_m = width_m
        self.is_valid = is_valid
        self.source = source

class Node:
    def save_image(self,request):
        # Save the image
        frames = []
        with self.lock_capture:
            for frame in self.frames:
                if frame is None:
                    #Can happen if images have not yet been captured, or
                    #if a camera is already being shut down, and None is captured
                    return
                frames.append(frame.copy())
        cv2.imwrite("/mnt/desktop/snapshots/near.jpg", frames[0])
        cv2.imwrite("/mnt/desktop/snapshots/far.jpg", frames[1])
        return []

    def callback_show(self,timer_event):
        frames = []
        with self.lock_capture:
            for frame in self.frames:
                if frame is None:
                    #Can happen if images have not yet been captured, or
                    #if a camera is already being shut down, and None is captured
                    return
                frames.append(frame.copy())
        img0=frames[0]
        img1=frames[1]
        image = cv2.vconcat([img0, img1])
        height, width, channels = img0.shape
        text = "{} x {}, {} channels".format( width, height, channels)
        cv2.putText(image, text,
                    (40, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 0),
                    2)

        image = cv2.resize(image, (0, 0), fx=0.3, fy=0.3)
        cv2.imshow('Image', image)
        cv2.waitKey(1)

    def callback_detect(self,timer_event):
        #rospy.loginfo(rospy.get_name() + " capturing image.")
        current_time = rospy.Time.now()
        frames = []
        frame_times = []
        with self.lock_capture:
            frame_times=self.frame_times[:]
            for frame in self.frames:
                if frame is None:
                    #Can happen if images have not yet been captured, or
                    #if a camera is already being shut down, and None is captured
                    return
                frames.append(frame.copy())
        time_near=frame_times[0]
        img_near=frames[0]
        time_far=frame_times[1]
        img_far=frames[1]
        #rospy.loginfo(rospy.get_name() + " images captured. Running detection")
        #rospy.loginfo(rospy.get_name() + " image shapes are {}, {}".format(str(np.shape(img0)),str(np.shape(img1))))
        #Detect far camera
        detections = []
        detections0 = []
        detections1 = []
        if debug_detect:
            detections1 = self.detect_from_image(False, img_far,time_far)
            #Detect near camera
            detections0 = self.detect_from_image(True, img_near,time_near)
            #rospy.loginfo(rospy.get_name() + " completed image detection")
            # publish the message
            for d in detections0+detections1:
                if d.is_valid:
                    position = Point(x=d.x_m,y=d.y_m,z=0.0)
                    color = self.extract_color(d,img_near,img_far)
                    det=singledetection(position=position, color=color, source=d.source,stamp=d.time)
                    detections.append(det)
        #rospy.loginfo(rospy.get_name() + " publishing detection")
        detectionarr = detectionarray(detections=detections)
        detectionarr.header.frame_id="camera_link"
        self.detectionarray_pub.publish(detectionarr)
        #rospy.loginfo(rospy.get_name() + " detection published")


        if current_time>self.last_image_sent+rospy.Duration.from_sec(1.0/stream_fps):
            self.last_image_sent=current_time
            # Stream an image (for vizualization only)
            img_far = self.display_detections(img_far, detections1)
            img_near = self.display_detections(img_near, detections0)


            img = cv2.vconcat([img_far, img_near])
            img = cv2.resize(img, (0, 0), fx=img_resize, fy=img_resize)
            if debug_show:
                cv2.imshow('Detection', img)
                cv2.waitKey(1)
                
            detimage = detectionimage()
            detimage.imageShape=list(img.shape)
            detimage.imageBase64= base64.b64encode(img.tobytes()).decode()
            self.detectionimage_pub.publish(detimage)

    def extract_color(self,d,img_near,img_far):
        img = img_near if d.source == 'NEAR' else img_far
        c = np.array(cv2.mean(img[d.y_min_pix:d.y_max_pix, d.x_min_pix: d.x_max_pix])) / 255
        return ColorRGBA(b=c[0], g=c[1], r=c[2], a=1.0)  # Array is BGR


    def detect_from_image(self,is_near, img, time):
        im_height, im_width, _ = img.shape
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        input_tensor = np.expand_dims(img, 0)
        det = self.detect_fn(input_tensor)

        bboxes = det['detection_boxes'][0].numpy()
        bclasses = det['detection_classes'][0].numpy().astype(np.int32)
        bscores = det['detection_scores'][0].numpy()
        detections = self.extract_detections(time, is_near, bboxes, bclasses, bscores, im_width, im_height)
        return detections

    #Returns array of Detection:
    # x_min (pixels)
    # y_min (pixels)
    # x_max (pixels)
    # y_max (pixels)
    # label (string)
    # score (double 0-1)
    # x_m (meters) of bottom left corner
    # y_m (meters) of bottom left corner
    # width_m (meters) width of detection (can be used to drop invalid detections)

    def extract_detections(self,time, is_near, bboxes, bclasses, bscores, im_width, im_height):
        detections = []
        for idx in range(len(bboxes)):
            if bscores[idx] >= detect_threshold:
                y_min_pix = int(bboxes[idx][0] * im_height)
                x_min_pix = int(bboxes[idx][1] * im_width)
                y_max_pix = int(bboxes[idx][2] * im_height)
                x_max_pix = int(bboxes[idx][3] * im_width)
                if is_near:
                    x_m, y_m = self.coord.near_pixel_to_m(x_min_pix, y_max_pix)  # Bottom left corner
                    x_m_br, y_m_br = self.coord.near_pixel_to_m(x_max_pix, y_max_pix)  # Bottom right corner
                else:
                    x_m, y_m = self.coord.far_pixel_to_m(x_min_pix, y_max_pix)
                    x_m_br, y_m_br = self.coord.far_pixel_to_m(x_max_pix, y_max_pix)  # Bottom right corner

                width_m = y_m - y_m_br
                label = class_labels[int(bclasses[idx])-1]
                score=float(bscores[idx])
                #bbox.append([x_min, y_min, x_max, y_max, label, score,x_cm, y_cm ])
                source = "NEAR" if is_near else "FAR"
                d = Detection( time, source, x_min_pix,  y_min_pix,  x_max_pix, y_max_pix, label,  score,  x_m,  y_m, width_m, True)
                d.is_valid = self.is_plausible(d)
                detections.append(d)
        self.mark_overlaps_invalid(detections)
        return detections

    # returns a list containing only the detections which are plausible (e.g. no very large objects which are bigger
    # then a lego brick
    def is_plausible(self,detection):
        is_near = detection.source == 'NEAR'
        ratio_y_to_x = (detection.y_max_pix-detection.y_min_pix) /(detection.x_max_pix-detection.x_min_pix)
        if not is_near and (ratio_y_to_x > 1.2 or ratio_y_to_x < 0.5):
            #rospy.loginfo(rospy.get_name() + " dropping unplausible detection in far camera (ratio is {})".format(ratio_y_to_x))
            return False
        if is_near and (ratio_y_to_x > 1.7 or ratio_y_to_x < 0.7):
            #rospy.loginfo(rospy.get_name() + " dropping unplausible detection in near camera (ratio is {})".format(ratio_y_to_x))
            return False
        if not is_near and (detection.width_m > 0.07 or detection.width_m < 0.02):
            #rospy.loginfo(rospy.get_name() + " dropping unplausible detection in far camera, width is {}m".format(width_m))
            return False
        if is_near and (detection.width_m > 0.07 or detection.width_m < 0.02):
            #rospy.loginfo(rospy.get_name() + " dropping unplausible detection in near camera, width is {}m".format(width_m))
            return False
        return True


    # marks overlapping detections invalid (overlapping area > threshold)
    def mark_overlaps_invalid(self,detections):
        for i in range(len(detections)):
            for j in range(i+1,len(detections)):
                overlap = self.compute_overlap(detections[i],detections[j])
                rospy.loginfo(rospy.get_name() + " overlap is {}".format(overlap))
                if overlap > OVERLAP_THRESHOLD:
                    rospy.loginfo(rospy.get_name() + " found overlap, dropping one")
                    detections[j].is_valid = False

    #Finds the percentage overlap of two rectangles
    def compute_overlap(self, detection1, detection2):
        left=max(detection1.x_min_pix,detection2.x_min_pix) #the 'righter' of the two left borders
        right=min(detection1.x_max_pix,detection2.x_max_pix) #the 'lefter' of the two right borders
        up=max(detection1.y_min_pix,detection2.y_min_pix) #the 'lower' (i.e. max y) of the two upper borders
        low=min(detection1.y_max_pix,detection2.y_max_pix) #the 'higher' (i.e. min y) of the two lower borders
        if up>low:
            return 0 #No overlap
        if right<left:
            return 0  # No overlap
        area1=(detection1.x_max_pix-detection1.x_min_pix)*(detection1.y_max_pix-detection1.y_min_pix)
        area2=(detection2.x_max_pix-detection2.x_min_pix)*(detection2.y_max_pix-detection2.y_min_pix)
        area=max(area1,area2)
        area_o=(right-left)*(low-up)
        return area_o/area

    def compare_validity(self,d):
        #valid entries to the end of the list, i.e. consider valid entires greater than invalid entries
        return 1 if d.is_valid else 0

    def display_detections(self,image, detections):
        img = image.copy()
        color_valid = (0, 255, 0)    #Color to be used for a valid detection
        color_invalid = (0, 0, 255)  #Color to be used for an invalid detection
        # sort to have invalid first, such that valid overlaps invalid in image
        det_sorted=detections[:]
        det_sorted.sort(key=self.compare_validity)

        for idx in range(len(det_sorted)):
            d = det_sorted[idx]
            color = color_valid if d.is_valid else color_invalid
            text = "{0}: {1:0.1f}%, {2:1d} x {3:1d} pix".format(d.label, d.score * 100, d.x_max_pix - d.x_min_pix,
                                                                d.y_max_pix - d.y_min_pix)
            coord_pix = "x = {0:1d}, y = {1:1d} pix".format(d.x_min_pix, d.y_max_pix)
            coord_m = "x = {0:.3f}, y = {1:.3f} m, w= {2:.3f} cm".format(d.x_m, d.y_m,d.width_m)
            cv2.rectangle(img, (d.x_min_pix, d.y_min_pix), (d.x_max_pix, d.y_max_pix), color, 4)
            cv2.rectangle(img, (d.x_min_pix, d.y_min_pix - 37), (d.x_min_pix, d.y_min_pix), (255, 255, 255), 4)
            cv2.putText(img, text, (d.x_min_pix + 5, d.y_min_pix - 27), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (127, 255, 0), 1)
            cv2.putText(img, coord_pix, (d.x_min_pix + 5, d.y_min_pix - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0),1)
            cv2.putText(img, coord_m, (d.x_min_pix + 5, d.y_min_pix - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0),1)
        return img

    def callback_readimg0(self,timer_event):
        self.readimg(0,timer_event)

    def callback_readimg1(self,timer_event):
        self.readimg(1,timer_event)

    def readimg(self,cap_idx,timer_event):
        now = rospy.Time.now()
        grabbed, frame = self.video_captures[cap_idx].read()
        with self.lock_capture:
            self.frames[cap_idx] = frame
            self.frame_times[cap_idx] = now

    def shutdownhook(self):
        rospy.loginfo(rospy.get_name() + " stopping the cameras")
        for video_capture in self.video_captures:
            video_capture.release()
        rospy.loginfo(rospy.get_name() + " cameras stopped.")

    # Couple of observations:
    # 1. Any sensor-mode other than 3 crops the image
    # 2. If two pipes are opened in the same program, the first pipe seems to be delayed.
    def gstreamer_pipeline(self,
            sensor_id,
            sensor_mode,
            width,
            height,
            framerate,
            flip_method,
    ):
        return ("nvarguscamerasrc sensor-id={} sensor-mode={} ! "
            "video/x-raw(memory:NVMM), width=(int){}, height=(int){}, format=(string)NV12, framerate=(fraction){}/1 ! "
            "nvvidconv flip-method={} ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! "
            "appsink"
                ).format(sensor_id,sensor_mode,width,height,framerate,flip_method)


    def start_cameras(self,flips):
        self.video_captures = []
        for i in range(len(flips)):
            gstreamer_pipeline_string=self.gstreamer_pipeline(
                sensor_id=i,
                sensor_mode=3,
                flip_method=flips[i],
                height=cam_height,
                width=cam_width,
                framerate=cam_fps
            )
            video_capture = cv2.VideoCapture(gstreamer_pipeline_string, cv2.CAP_GSTREAMER)
            if not video_capture.isOpened():
                raise RuntimeError('Could not open video capture')
            self.video_captures.append(video_capture)

    # Power on and prepare for general usage.
    # This will activate the device and take it out of sleep mode (which must be done
    # after start-up). This function also sets both the accelerometer and the gyroscope
    # to default range settings, namely +/- 2g and +/- 250 degrees/sec.
    def __init__(self):
        rospy.init_node('detection_node')
        rospy.on_shutdown(self.shutdownhook)
        self.coord = CameraCoordinates()
        self.lock_capture = threading.Lock()
        self.video_captures = []
        self.frames = [None,None]
        self.frame_times=[None,None]
        flips=[2,2]
        self.start_cameras(flips)
        rospy.loginfo(rospy.get_name() + " Opened cameras")
        self.last_image_sent = rospy.Time.now()
        # Load model
        if debug_detect:
            rospy.loginfo(rospy.get_name() + " Loading model...")
            tensorflow.keras.backend.clear_session()
            start = rospy.Time.now()
            self.detect_fn = tensorflow.saved_model.load(path_to_checkpoint)
            rospy.loginfo(rospy.get_name() + " ...model loaded in {} seconds".format((rospy.Time.now()-start)/1000000000))

        # Initialize the node and name it.
        self.detectionarray_pub = rospy.Publisher("detectionarray", detectionarray, queue_size=10)
        self.detectionimage_pub = rospy.Publisher("detectionimage", detectionimage, queue_size=2)
        if debug_detect:
            rospy.Timer(Duration.from_sec(1.0/detect_fps), self.callback_detect)
        if debug_show:
            rospy.Timer(Duration.from_sec(1.0/cam_fps), self.callback_show)
        # Read images from cameras at a bit faster than the frame rate (to never have buffer overrun)
        # Doing this in two separate threads (timers) guarantees that we have full speed on both channels
        rospy.Timer(Duration.from_sec(1.0/(cam_fps*1.1)), self.callback_readimg0)
        rospy.Timer(Duration.from_sec(1.0/(cam_fps*1.1)), self.callback_readimg1)
        rospy.Service('detection_node/save_image', Empty, self.save_image) #Can be used to debug
        rospy.loginfo(rospy.get_name() + " started detection_node.")
        rospy.spin()




# Main function.
if __name__ == '__main__':
    node = Node()

