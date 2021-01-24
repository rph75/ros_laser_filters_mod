import motor_controller as mc
import cv2
import time
import numpy as np
import tensorflow as tf
import argparse
import math
import coordinates
from enum import Enum
from collections import namedtuple
import ina3221
import gripper_controller as gc

class_labels = ['front','top'] #TODO: Read from label map instead
path_to_checkpoint='/home/roman/models/bricks_model/saved_model'
#path_to_checkpoint='/home/roman/models/efficientdet_d0_coco17_tpu-32/saved_model'
pipeline_cam='nvarguscamerasrc sensor_id={} ! video/x-raw(memory:NVMM), width={}, height={}, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=True'
#pipeline_videoout='appsrc ! videoconvert ! x264enc key-int-max=12 byte-stream=true tune="zerolatency" ! mpegtsmux ! tcpserversink host=0.0.0.0 port=5000 sync=false'
pipeline_videoout='appsrc ! videoconvert ! x264enc key-int-max=12 byte-stream=true ! mpegtsmux ! filesink location=/home/roman/test.mts '
#pipeline_videoout='appsrc ! autovideoconvert ! omxh265enc ! matroskamux ! filesink location=/home/roman/test.mkv '
cam_width = 800
cam_height = 600

left_range_x = (int)(cam_width * 1/4)    # Left boundary for cm range
right_range_x = (int)(cam_width * 3/4)		# Right boundary for cm range
top_range_y = (int)(cam_height * 1/3)		# Top boundary for cm range
bottom_range_y = (int)(cam_height * 2/3)	# Lower boundary for cm range

fps_videoout = 5
fourcc_videoout = 0 # cv2.VideoWriter_fourcc(*"mp4v")
detect_threshold = 0.6

h_cm=1.9 #Height of brick

timePID = 15  # Run PID once every x ms
kp = 600
ki = 20
kd = 0
quotient = 100  # P,I,D are scaled by factor 100
timePID_pos = 200
kp_pos = 0
ki_pos = 0
kd_pos = 0
quotient_pos = 10

critical_voltage = 3.4 * 3 #Critical voltage below which battery warning appears (3.2 V is absolute lower level)

near_speed_cm_per_sec = 4.0
far_speed_cm_per_sec = 16.0
near_speed_distance_cm = 40.0 # Anything closer to this distance will be approached with near speed

detection_queue_size = 5 #Need x frames with detections to be stable
max_distance_pix = 8    #Max distance in pixels from average lower left corner of last x frames for a detection queue to be considered stable
gripper_distance = 0.3  #Max distance in cm from gripper in order to pick up the piece

#gripper_pos_down = 0.895 # Servo position when gripper is down
gripper_pos_down = 0.95 # Servo position when gripper is down
gripper_pos_up = 0.25 # Servo position when gripper is up (default position)
gripper_move_time= 1500 # ms to move gripper from one position to another
gripper_grip_time= 1500 # ms to open/close gripper

Detection = namedtuple("Detection", "x_min_pix y_min_pix x_max_pix y_max_pix label score x_cm y_cm")

class Mode(Enum):
	detect_near = 1
	detect_near_wait_time = 2
	detect_near_wait_still = 3
	detect_far = 10
	detect_far_wait_time = 11
	gripper_down = 20
	gripper_close = 21
	gripper_up = 22
	gripper_open = 23

class Still(Enum):
	unstable = 1 #Image is not stable over past X frames
	no_detections = 2 #No detections found over past X frames
	stable = 3 #Found a stable detection

def detect_from_image(is_near, img,detect_fn):
	im_height, im_width, _ = img.shape
	# Expand dimensions since the model expects images to have shape: [1, None, None, 3]
	input_tensor = np.expand_dims(img, 0)
	timestamp = time.time()
	det = detect_fn(input_tensor)
	elapsed_time = round((time.time() - timestamp) * 1000)  # ms

	bboxes = det['detection_boxes'][0].numpy()
	bclasses = det['detection_classes'][0].numpy().astype(np.int32)
	bscores = det['detection_scores'][0].numpy()
	detections = extract_detections(is_near, bboxes, bclasses, bscores, im_width, im_height)

	return detections, elapsed_time

#Returns array of Detection:
# x_min (pixels)
# y_min (pixels)
# x_max (pixels)
# y_max (pixels)
# label (string)
# score (double 0-1)
# x_cm (cm or None) of bottom left corner
# y_cm (cm or None) of bottom left corner

def extract_detections(is_near, bboxes, bclasses, bscores, im_width, im_height):
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
			d = Detection(x_min_pix=x_min_pix, y_min_pix=y_min_pix, y_max_pix=y_max_pix, x_max_pix=x_max_pix, label=label, score=score, x_cm=x_cm, y_cm=y_cm)
			detections.append(d)
	return detections


def display_detections(image, detections, best):
	img = image.copy()
	for idx in range(len(detections)):
		d=detections[idx]
		text = "{0}: {1:0.1f}%, {2:1d} x {3:1d} pix".format(d.label,d.score*100,d.x_max_pix-d.x_min_pix,d.y_max_pix-d.y_min_pix)
		coord_pix = "x = {0:1d}, y = {1:1d} pix" .format(d.x_min_pix,d.y_max_pix)
		if d.x_cm is not None and d.y_cm is not None:
			coord_cm = "x = {0:.1f}, y = {1:.1f} cm" .format(d.x_cm,d.y_cm)
		else:
			coord_cm = "N/A"

		if idx == best:
			color = (0, 0, 255)
		else:
			color = (0, 255, 0)
		cv2.rectangle(img, (d.x_min_pix, d.y_min_pix), (d.x_max_pix, d.y_max_pix), color, 1)
		cv2.rectangle(img, (d.x_min_pix, d.y_min_pix - 37), (d.x_min_pix, d.y_min_pix), (255, 255, 255), -1)
		cv2.putText(img, text, (d.x_min_pix + 5, d.y_min_pix - 27), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (127, 255, 0), 1)
		cv2.putText(img, coord_pix, (d.x_min_pix + 5, d.y_min_pix - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
		cv2.putText(img, coord_cm, (d.x_min_pix + 5, d.y_min_pix - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
	return img


def find_best_detection_far(detections):
	best = -1
	shortest_dist = 0
	for i in range(0, len(detections)):
		d = detections[i]
		if best < 0 or d.y_cm < shortest_dist:
			best = i
			shortest_dist = d.y_cm
	return best


# Find the detection which is closest to the gripper
# returns -1 if no detection found
def find_best_detection_near(detections):
	best = -1
	shortest_dist_gripper = 0
	for i in range(0, len(detections)):
		d = detections[i]
		if d.x_cm is not None and d.y_cm is not None:
			dist = coordinates.distance(d.x_cm, d.y_cm, coordinates.gripper_x_cm, coordinates.gripper_y_cm)
			if best < 0 or dist < shortest_dist_gripper:
				best = i
				shortest_dist_gripper = dist
	if best < 0:
		# Couldn't find any proper detection baesd on cm - try now to find detection that may be outside the conversion range
		# by finding the object which is closest to mid of lower end of image
		shortest_dist_pix = 0
		for i in range(0, len(detections)):
			d = detections[i]
			if d.x_cm is None or d.y_cm is None:
				x_pix = (d.x_min_pix+d.x_max_pix)/2
				y_pix = d.y_max_pix
				dist = coordinates.distance(x_pix, y_pix, cam_width /2, cam_height)
				if best < 0 or dist < shortest_dist_pix:
					best = i
					shortest_dist_pix = dist
	return best

# Checks if the detection_queue contains a still detection, i.e. a detection that was not moving
def find_still(detection_queue):
	if len(detection_queue) < detection_queue_size:
		return None, None, Still.unstable

	#Find average position of lower left corner:
	count = 0
	x_sum = 0
	y_sum = 0
	for d in detection_queue:
		if d is not None:
			count = count +1
			x_sum = x_sum + d.x_min_pix
			y_sum = y_sum + d.y_max_pix

	if count == 0:
		# None of the images has a detection
		return None, None, Still.no_detections

	if count<detection_queue_size:
		# Not all images had a detection
		return None, None, Still.unstable

	x_avg = x_sum / count
	y_avg = y_sum / count

	# Check if all detections are close enough to average:
	for d in detection_queue:
		if d is not None:
			dist = coordinates.distance(d.x_min_pix, d.y_max_pix, x_avg, y_avg)
			if dist > max_distance_pix:
				# This point is too far from the average
				return None, None, Still.unstable

	# Found a stable detection
	return x_avg, y_avg, Still.stable


def run_detection(save_output):
	# Init power control
	pc = ina3221.SDL_Pi_INA3221(addr=0x40)

	# Init motor controller
	mc.initController()
	mc.upload_pid(timePID, quotient, kp, ki, kd, timePID_pos, quotient_pos, kp_pos, ki_pos, kd_pos)

	# Move gripper to default position (which is up)
	gc.initGripper(gripper_pos_up) #Start with up position
	gc.open_gripper(gripper_grip_time)

	cap0 = None
	cap1 = None
	video_out = None
	mode=Mode.detect_far

	try:

		# Load model
		print('loading model')
		tf.keras.backend.clear_session()
		now=time.time()
		detect_fn = tf.saved_model.load(path_to_checkpoint)
		print('model loaded in %d seconds '%((int)(time.time()-now)))

		# Open camera streamers
		cap0 = cv2.VideoCapture(pipeline_cam.format(0,cam_width,cam_height))
		cap1 = cv2.VideoCapture(pipeline_cam.format(1,cam_width,cam_height))
		if save_output:
			video_out = cv2.VideoWriter(pipeline_videoout, fourcc_videoout, fps_videoout, (cam_width * 2, cam_height))

		near_detections = [] 	# Array containing the near detections
		time_last_detection = 0 # Time when a detection was found the last time
		while cap0.isOpened() and cap1.isOpened():
			ret0, img0 = cap0.read()
			if not ret0:
				break
			ret1, img1 = cap1.read()
			if not ret1:
				break

			far_detection = None
			best = -1
			if mode == Mode.detect_far or mode == Mode.detect_far_wait_time:
				# TODO: Could save battery by not running this in Mode.detect_far_wait_time, as the detection wouldn't be used anyway
				#  (but doesn't look as cool on camera as the detection is missing)
				detections1, elapsed_time = detect_from_image(False, img1,detect_fn)
				best=find_best_detection_far(detections1)
				img1 = display_detections(img1, detections1, best)
				if best >= 0:
					far_detection = detections1[best]
			else:
				detections0, elapsed_time = detect_from_image(True, img0, detect_fn)
				best = find_best_detection_near(detections0)
				img0 = display_detections(img0, detections0, best)
				color = (140,140,140)
				cv2.line(img0, (0,top_range_y), (cam_width,top_range_y), color, 1)
				cv2.line(img0, (0,bottom_range_y), (cam_width,bottom_range_y), color, 1)
				cv2.line(img0, (left_range_x,0), (left_range_x,cam_height), color, 1)
				cv2.line(img0, (right_range_x,0), (right_range_x,cam_height), color, 1)

				near_detections.append(detections0[best] if best>=0 else None)
				if len(near_detections) > detection_queue_size:
					del near_detections[0]
			if best >= 0:
				time_last_detection = time.time()

			img = cv2.hconcat([img0, img1])
			fps = round(1000. / elapsed_time, 1)
			fps_txt = str(fps) + " FPS"
			cv2.putText(img, fps_txt, (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
			power_brain = pc.getPower_W(1)
			power_total = pc.getPower_W(3)
			voltage = pc.getBusVoltage_V(3)
			power_txt = "Brain {0:2.1f} of {1:2.1f}W".format(power_brain, power_total)
			cv2.putText(img, power_txt, (25, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
			voltage_txt = "Batt {0:2.2f} V".format(voltage)
			if voltage > critical_voltage:
				cv2.putText(img, voltage_txt, (25, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
			else:
				cv2.putText(img, voltage_txt, (25, 100), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 3)

			if video_out is not None:
				video_out.write(img)

			cv2.imshow('Detection', img)
			key = cv2.waitKey(1)
			if key == 27:
				break

			if mode == Mode.detect_near:
				x_pix, y_pix, found_still = find_still(near_detections)
				if found_still == Still.no_detections:
					# nothing found near us go detect far
					if time_last_detection + 3.0 < time.time():
						# No detection in x seconds - neither near nor far
						#Turn robot by a bit, then wait, then start looking close again
						alpha_deg = -10 # Turn by 10 degrees left
						fwd_left_cm, fwd_right_cm = coordinates.move_robot(0, alpha_deg / 180 * math.pi)
						time_ms = mc.forward_position(fwd_left_cm, fwd_right_cm, near_speed_cm_per_sec)
						wait_until = time.time() + time_ms / 1000.0 + 1.0
						mode = Mode.detect_far_wait_time
					else:
						mode = Mode.detect_far
				elif found_still == Still.unstable:
					# Stay in detect close mode, as the image is unstable
					pass
				elif found_still == Still.stable:
					x_cm, y_cm = coordinates.near_pixel_to_cm(x_pix, y_pix)
					if x_cm is not None and y_cm is not None:
						if coordinates.distance(x_cm, y_cm, coordinates.gripper_x_cm, coordinates.gripper_y_cm) < gripper_distance:
							# Grip the object
							print("Moving gripper forward")
							gc.go_to_position(gripper_pos_down,gripper_move_time)
							wait_until = time.time() + gripper_move_time/1000
							mode = Mode.gripper_down
						else:
							# Distance too far - readjust the robot position
							# TODO: looks like angle calculation is not correct - test this out and improve!
							fwd_left_cm, fwd_right_cm = coordinates.move_gripper_to_pos(x_cm, y_cm)
							time_ms = mc.forward_position(fwd_left_cm, fwd_right_cm, near_speed_cm_per_sec)
							wait_until = time.time() + time_ms/1000.0
							mode = Mode.detect_near_wait_time
					else:
						# Object is outside the measurable place - move forward / backward / sidewise by a fixed amount
						# TODO: Better place for this code
						forward_cm = 0
						alpha_deg = 0
						if y_pix > bottom_range_y:
							# Object is in lower area - move backward
							forward_cm = -5.0
						elif y_pix < top_range_y:
							# Object is in upper area - move forward
							forward_cm = 5.0
						if x_pix < left_range_x:
							#Object is left - move left
							alpha_deg = -15
						elif x_pix > right_range_x:
							#Object is right - move right
							alpha_deg = 15
						if alpha_deg == 0 and forward_cm == 0:
							# Be safe and handle case where nothing would move at all - back off 10 cm
							forward_cm = -10
						fwd_left_cm, fwd_right_cm = coordinates.move_robot(forward_cm,alpha_deg / 180 * math.pi)
						time_ms = mc.forward_position(fwd_left_cm, fwd_right_cm, near_speed_cm_per_sec)
						wait_until = time.time() + time_ms / 1000.0
						mode = Mode.detect_near_wait_time

			elif mode == Mode.detect_near_wait_time:
				if time.time() >= wait_until:
					near_detections = []  # Clean out detection queue - need to take full set of new images as robot moved
					mode = Mode.detect_near
			elif mode == Mode.detect_far:
				if far_detection is None:
					# Nothing found in the distance - try close
					mode = Mode.detect_near
				else:
					# Approach the far detection
					# TODO: Remove cm from detection class, as it is never used anyway?
					x_cm, y_cm = coordinates.far_pixel_to_cm(far_detection.x_min_pix, far_detection.y_max_pix)
					if x_cm is None:
						# Could not evaluate exact x position -> just turn a bit
						# TODO: Should turn more for objects closer to robot
						# TODO: Better place for this code
						if far_detection.x_min_pix < 350:
							# It is on the left side -> turn left
							alpha_deg = -20
						elif far_detection.x_min_pix > 520:
							# It is on the right side -> turn right
							alpha_deg = 20
						else:
							# It is roughly in the middle -> do not turn
							alpha_deg = 0
						fwd_left_cm, fwd_right_cm = coordinates.move_robot(y_cm, alpha_deg / 180 * math.pi)
					else:
						fwd_left_cm, fwd_right_cm = coordinates.move_gripper_to_pos(x_cm, y_cm)

					speed = far_speed_cm_per_sec if y_cm > near_speed_distance_cm else near_speed_cm_per_sec
					time_ms = mc.forward_position(fwd_left_cm, fwd_right_cm, speed)
					wait_until = time.time() + (time_ms / 1000.0) * 0.3 # Wait only a portion of the required time, then reassess the position with next capture
					mode = Mode.detect_far_wait_time
			elif mode == Mode.detect_far_wait_time:
				if time.time() >= wait_until:
					mode = Mode.detect_far
			elif mode == Mode.gripper_down:
				if time.time() >= wait_until:
					# Gripper moved down - close gripper
					print("Closing gripper")
					gc.close_gripper(gripper_grip_time)
					wait_until = time.time() + gripper_grip_time / 1000
					mode = Mode.gripper_close
			elif mode == Mode.gripper_close:
				if time.time() >= wait_until:
					# Gripper is closed - move gripper backward
					print("Moving gripper backward")
					gc.go_to_position(gripper_pos_up,gripper_move_time)
					wait_until = time.time() + gripper_move_time / 1000
					mode = Mode.gripper_up
			elif mode == Mode.gripper_up:
				if time.time() >= wait_until:
					# Gripper moved up - open gripper
					print("Opening gripper")
					gc.open_gripper(gripper_grip_time)
					wait_until = time.time() + gripper_grip_time / 1000
					mode = Mode.gripper_open
			elif mode == Mode.gripper_open:
				if time.time() >= wait_until:
					# Gripper moved backward and is now fully open
					near_detections = []  # Clean out detection queue - need to take full set of new images as gripper moved
					mode = Mode.detect_near

	finally:
		# Cleanup
		gc.open_gripper(gripper_grip_time) #Go back to default position
		gc.go_to_position(gripper_pos_up, gripper_move_time) #Go back to default position
		if cap0 is not None:
			cap0.release()
		if cap1 is not None:
			cap1.release()
		if video_out is not None:
			video_out.release()
		cv2.destroyAllWindows()


parser = argparse.ArgumentParser()
parser.add_argument("save_output",help="Save output")
args=parser.parse_args()
save_output=True #todo
run_detection(save_output)


