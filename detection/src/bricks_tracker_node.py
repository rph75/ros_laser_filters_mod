#!/usr/bin/env python

import numpy as np
#import cv2 #Must be imported before importing tensorflow, otherwise some strange error god knows why
import threading
import rospy
import math
import tf
import operator
import camera_coord
from detection.msg import detection
from std_msgs.msg import Header,ColorRGBA
from geometry_msgs.msg import PointStamped, Point, PoseArray, Pose, PoseStamped, Quaternion, Twist, Vector3, Polygon, PolygonStamped, Point32
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal, MoveBaseAction
from detection.msg import singledetection ,detectionarray,detectionimage
from actionlib_msgs.msg import GoalID, GoalStatus
from motorctrl.srv import  motorctrl_writepos, motorctrl_readpos, motorctrl_grippercommand
from motorctrl.msg import motorctrl_writeentry
import actionlib
from rospy import Duration


marker_scale=Vector3(0.032,0.032,0.02)
forward=Quaternion(w=1.0,x=0,y=0,z=0)
text_scale=Vector3(0.0,0.0,0.05) #Only scale.z is used. scale.z specifies the height of an uppercase "A".
text_color=ColorRGBA(r=0.2,g=0.8,b=0.0,a=1.0)
cyl_color=ColorRGBA(r=1.0,g=1.0,b=1.0,a=0.5)
#Visbile areas, in base_link frame

def add_to_point(point,offset):
    return Point(x=point.x+offset[0],y=point.y+offset[1],z=point.z+offset[1])

# Gripper area, relative to gripper coordinate frame
GRIPPER_POLY = [Point(x=-0.05, y=-0.11, z=0.0), Point(x=0.25, y=-0.11, z=0.0), Point(x=0.25, y=0.11, z=0.0), Point(x=-0.05, y=0.11, z=0.0)]

#near_poly = [Point(x=0.15, y=0.10, z=0.0), Point(x=0.35, y=0.10, z=0.0), Point(x=0.35, y=-0.10, z=0.0), Point(x=0.15, y=-0.10, z=0.0)]
#far_poly = [Point(x=0.40, y=0.20, z=0.0), Point(x=1.00, y=0.80, z=0.0), Point(x=1.00, y=-0.80, z=0.0), Point(x=0.40, y=-0.20, z=0.0)]
RESET_COUNT = 5 #Count number of detections when brick is missing until it is removed for good (to stabilize temporary misses)

BRICK_COLORS=[
    ColorRGBA(r=1.0, g=0.0, b=0.0,a=1.0),       # red
    ColorRGBA(r=0.0, g=1.0, b=0.0,a=1.0),       # green
    ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),      # blue
    ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),      # yellow
    #ColorRGBA(r=1.0, g=0.2, b=1.0, a=1.0),      # pink
    #ColorRGBA(r=0.0, g=1.0, b=0.5, a=1.0),      # bright-green
]

STATUS_IDLE = 0         #We're not doing pursuing any brick
STATUS_MOVINGBASE = 1   #A brick has been identified, and a goal has been sent to move base
STATUS_APPROACHING = 2  #A brick has been identified in the near camera, and we're now approaching the gripper

STATUS_GRIPPER_LOWERING = 10
STATUS_GRIPPER_RAISING = 11
STATUS_GRIPPER_CLOSING = 12
STATUS_GRIPPER_OPENING = 13

STATUS_STR=[
'PENDING',
'ACTIVE',
'PREEMPTED',
'SUCCEEDED',
'ABORTED',
'REJECTED',
'PREEMPTING',
'RECALLING',
'RECALLED',
'LOST',
]

GRIPPER_DIST = 0.15     #Distance from the brick when setting a goal

GRIPPER_TOLERANCE = 0.003  #Max distance in m from gripper in order to pick up the piece

GRIPPER_POS_DOWN = 0.95 # Servo position when gripper is down
GRIPPER_POS_UP = 0.25 # Servo position when gripper is up (default position)
GRIPPER_MOVE_TIME= Duration.from_sec(2.5) # time to move gripper from one position to another
GRIPPER_GRIP_TIME= Duration.from_sec(1.5) # time to open/close gripper
GOAL_STATUS_LOG_INTERVAL = Duration.from_sec(5.0)
enc_per_m=11500.0 #encoder units per cm #TODO: centralize this value (shared with odom_data)

#Tolerance of base_link to waypoint before the target waypoint is updated to the next one
WAYPOINT_TOLERANCE = 0.5
#Defines a path which the robot should follow if there is no brick around to collect
IDLE_WAYPOINTS = [
  Pose(position=Point(x= 1.38682508469,y= -2.93354725838,z= 0.0),orientation=Quaternion(x= 0.0,y= 0.0,z=-0.688960795656,w= 0.724798607924)),
  Pose(position=Point(x= 5.36411476135,y=-5.82175683975,z= 0.0),orientation=Quaternion(x= 0.0,y=0.0,z= 0.0202973180118,w= 0.99979398822)),
  Pose(position=Point(x= 6.97185945511,y=-8.34920787811,z= 0.0),orientation=Quaternion(x= 0.0,y=0.0,z= 0.711992995144,w= 0.702186566994)),
  Pose(position=Point(x= 5.53210258484,y=-1.2927801609,z= 0.0),orientation=Quaternion(x= 0.0,y=0.0,z= -0.999803428543,w= 0.0198268573953)),
  Pose(position=Point(x= 1.45372509956,y=-1.14178466797,z= 0.0),orientation=Quaternion(x= 0.0,y=0.0,z= 0.948261213466,w=  0.317491214109)),
  Pose(position=Point(x= 2.47082614899,y= 4.60656261444,z= 0.0),orientation=Quaternion(x= 0.0,y=0.0,z= 0.00465937461482,w= 0.999989145055)),
  Pose(position=Point(x= 3.71764445305,y=4.6676235199,z= 0.0),orientation=Quaternion(x= 0.0,y=0.0,z= -0.68848681591,w= 0.72524885682)),
  Pose(position=Point(x= 6.06605052948,y= -0.270807504654,z= 0.0),orientation=Quaternion(x= 0.0,y=0.0,z= -0.683218452874,w= 0.730214040986)),
]


class Brick:
    def __init__(self, position, color, radius, count, source, id):
        self.id=id
        self.position=position
        self.color=color
        self.radius=radius
        self.count=count
        self.source=source

    def dec_count(self):
        self.count=self.count-1

class Node:
    # Power on and prepare for general usage.
    # This will activate the device and take it out of sleep mode (which must be done
    # after start-up). This function also sets both the accelerometer and the gyroscope
    # to default range settings, namely +/- 2g and +/- 250 degrees/sec.
    def __init__(self):
        rospy.init_node('bricks_tracker_node')
        self.lock = threading.Lock()
        self.wait_until = rospy.Time.now()
        self.log_next_goal_status = rospy.Time.now()
        self.approaching_valid_pos_counter = 0
        self.status = STATUS_IDLE
        self.brick_goal = None
        self.brick_id = 1
        self.target_pose = None
        self.next_idle_waypoint = None
        self.pool = []
        self.move_base_action = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.listener = tf.TransformListener()
        rospy.wait_for_service('/motorctrl_service/writepos')
        self.writepos_proxy = rospy.ServiceProxy('motorctrl_service/writepos', motorctrl_writepos)
        self.readpos_proxy = rospy.ServiceProxy('motorctrl_service/readpos', motorctrl_readpos)
        self.gripper_proxy = rospy.ServiceProxy('motorctrl_service/grippercommand', motorctrl_grippercommand)
        rospy.Subscriber("detectionarray", detectionarray, self.callback,queue_size=2) #TODO: Cannot process fast enough, therefore dropping - is this good?
        self.markers_pub = rospy.Publisher("bricks", MarkerArray, queue_size=10)
        self.stable_area_pub = rospy.Publisher("stable_area", PolygonStamped, queue_size=10)
        self.gripper_area_pub = rospy.Publisher("gripper_area", PolygonStamped, queue_size=10)
        self.idle_waypoints_pub = rospy.Publisher("idle_waypoints", PoseArray, queue_size=10)
        interval = rospy.Duration.from_sec(0.5)
        rospy.Timer(interval, self.callback_tim)
        self.move_base_action.cancel_goal() #Cancel any previous goal, such that when this node gets started we are clean
        rospy.loginfo(rospy.get_name() + " started bricks_tracker_node.")
        rospy.spin()

    def callback_tim(self,timer_event):
        ps_stable=PolygonStamped(header=Header(frame_id="camera_link"), polygon=self.convert_poly_to_msg(camera_coord.STABLE_POLY))
        self.stable_area_pub.publish(ps_stable)

        ps_gripper=PolygonStamped(header=Header(frame_id="gripper"), polygon=self.convert_poly_to_msg(GRIPPER_POLY))
        self.gripper_area_pub.publish(ps_gripper)

        self.idle_waypoints_pub.publish(PoseArray(header=Header(frame_id="map"),poses=IDLE_WAYPOINTS))

        current_time = rospy.Time.now()
        if current_time < self.wait_until:
            #Timer still ticking
            return

        # Now trace the stuff
        goal_status = self.move_base_action.get_state()
        if (current_time > self.log_next_goal_status):
            rospy.loginfo(rospy.get_name() + " move base goal status is {} ({}), state machine is in status {} ".format(goal_status,STATUS_STR[goal_status],self.status))
            self.log_next_goal_status=current_time+GOAL_STATUS_LOG_INTERVAL

        # Get the current robot position in the map frame
        self.listener.waitForTransform("map","gripper", current_time, rospy.Duration.from_sec(10.0))
        self.listener.waitForTransform("base_link","gripper", current_time, rospy.Duration.from_sec(10.0))
        #pose_map = self.listener.transformPose("map", PoseStamped(header=Header(frame_id = "base_link",stamp=current_time),pose=Pose(position=Point(x=0,y=0,z=0),orientation=Quaternion(0,0,0,1)))).pose;
        pose_gripper_map = self.listener.transformPose("map", PoseStamped(header=Header(frame_id = "gripper",stamp=current_time),pose=Pose(position=Point(x=0,y=0,z=0),orientation=Quaternion(0,0,0,1)))).pose;
        pose_gripper_base = self.listener.transformPose("base_link", PoseStamped(header=Header(frame_id = "gripper",stamp=current_time),pose=Pose(position=Point(x=0,y=0,z=0),orientation=Quaternion(0,0,0,1)))).pose;
        pose_base_map = self.listener.transformPose("map", PoseStamped(header=Header(frame_id = "base_link",stamp=current_time),pose=Pose(position=Point(x=0,y=0,z=0),orientation=Quaternion(0,0,0,1)))).pose;
        # Get the gripper window
        gripper_poly_map = self.transform_poly(GRIPPER_POLY, current_time, "gripper", "map")
        if self.status in [STATUS_IDLE,STATUS_APPROACHING,STATUS_MOVINGBASE]:
            # Check if the brick still exists and is still reachable - otherwise cancel goal (if there is any) and go back to idle status
            with self.lock:
                if self.brick_goal is not None and (self.brick_goal not in self.pool):
                    rospy.loginfo(rospy.get_name() + " brick {} is no longer a valid goal - cancelling ".format(self.brick_goal.id))
                    self.move_base_action.cancel_goal() #Cancel any current goal - just in case
                    self.brick_goal = None
                    self.status = STATUS_IDLE
                    return
        # Check if we see any object in the gripper area - if yes, grab it (no matter of what the current target object is)
        if self.status in [STATUS_IDLE,STATUS_MOVINGBASE]:
            with self.lock:
                #If there are >1 in the gripper area, find the one which is closest to the gripper (or closest to the base?)
                closest_brick = None
                for b in self.pool:
                    if self.ray_tracing(b.position, gripper_poly_map):
                        d = self.calc_distance(b,pose_gripper_map);
                        if closest_brick is None or d < distance:
                            distance = d
                            closest_brick = b
                # check if we found a brick within gripper area
                if closest_brick is not None:
                    rospy.loginfo(rospy.get_name() + " Found brick {} in gripper area - approach it ".format(closest_brick.id))
                    self.brick_goal = closest_brick
                    self.move_base_action.cancel_goal() #Cancel any current goal - just in case
                    self.approaching_valid_pos_counter = 0
                    self.status = STATUS_APPROACHING
                    return
        #
        if self.status == STATUS_IDLE:
            with self.lock:
                #Find the closest brick (if any) - and move towards it
                closest_brick = None
                for b in self.pool:
                    d = self.calc_distance(b, pose_gripper_map);
                    if closest_brick is None or d < distance:
                        distance = d
                        closest_brick = b
            # check if we found a closest brick
            if closest_brick is not None:
                rospy.loginfo(rospy.get_name() + " Closest brick is {}, sending goal".format(closest_brick.id))
                #self.next_idle_waypoint = None # reset idle path
                self.brick_goal = closest_brick
                self.target_pose = self.compute_base_target_pos(closest_brick,pose_gripper_map,pose_gripper_base)
                #rospy.loginfo(rospy.get_name() + " Going with base link to coordinates {}".format(target_pose_base_map))
                self.status = STATUS_MOVINGBASE
                #Send the action
                rospy.loginfo(rospy.get_name() + " Sending goal to move base")
                self.move_base_action.send_goal(MoveBaseGoal(target_pose=PoseStamped(header=Header(frame_id="map", stamp=current_time), pose=self.target_pose)))
                return
            else:
                # No bricks around - follow an idle path
                updated_waypoint = None
                if self.next_idle_waypoint is None:
                        #Find the closest waypoint from current position (this is for the very first time)
                        for waypoint in IDLE_WAYPOINTS:
                            if updated_waypoint is None or self.calc_distance(pose_base_map,waypoint) < self.calc_distance(pose_base_map,updated_waypoint):
                                updated_waypoint = waypoint
                        rospy.loginfo(rospy.get_name() + " found closest waypoint {}".format(updated_waypoint))
                elif goal_status != 1:
                    updated_waypoint = self.next_idle_waypoint
                    #rospy.loginfo(rospy.get_name() + " goal status is not active; resuming path to waypoint {}".format(updated_waypoint))
                elif self.calc_distance(pose_base_map,self.next_idle_waypoint) < WAYPOINT_TOLERANCE:
                    # We are quite close to the waypoint - update goal to next waypoint before the robot will start tricks to reach goal exactly
                    i = (IDLE_WAYPOINTS.index(self.next_idle_waypoint)+1) % len(IDLE_WAYPOINTS)
                    updated_waypoint = IDLE_WAYPOINTS[i]
                    #rospy.loginfo(rospy.get_name() + " close to waypoint {}, going to next waypoint {}, {}".format(self.next_idle_waypoint,i,updated_waypoint))

                if updated_waypoint is not None:
                    #Send target to next waypoint
                    #rospy.loginfo(rospy.get_name() + " going to waypoint {}".format(updated_waypoint))
                    self.next_idle_waypoint = updated_waypoint
                    self.move_base_action.send_goal(MoveBaseGoal(target_pose=PoseStamped(header=Header(frame_id="map", stamp=current_time),pose=updated_waypoint)))
                #
                return

        if self.status==STATUS_MOVINGBASE:
            # Check if we're healthy and moving
            if goal_status == GoalStatus.PENDING or goal_status == GoalStatus.ACTIVE:
                #Still moving
                # When getting close to target (brick_goal is in far camera, and close enough): Adjust the goal, as we're getting more precision
                target_pose = self.compute_base_target_pos(self.brick_goal,pose_gripper_map,pose_gripper_base)
                d = self.calc_distance(self.target_pose,target_pose);
                if d > 0.1: #TODO: Could make this dynamic and reduce the tolerance the closer we get
                    rospy.loginfo(rospy.get_name() + " Readjusting goal because previous target is {} m off".format(d))
                    self.target_pose = target_pose
                    self.move_base_action.send_goal(MoveBaseGoal(target_pose=PoseStamped(header=Header(frame_id="map", stamp=current_time), pose=self.target_pose)))
                return
            if goal_status != GoalStatus.SUCCEEDED:
                # Finished, but with error
                rospy.loginfo(rospy.get_name() + " Could not reach goal")
            # reached the goal or aborted - go back to idle mode (which will initate gripper if it is in the right place)
            else:
                rospy.loginfo(rospy.get_name() + " Goal reached")
            self.brick_goal = None
            self.status = STATUS_IDLE
            return

        if self.status == STATUS_APPROACHING:
            # Try placing the brick directly under the gripper, then wait a bit for getting a few stable shots
            if not self.ray_tracing(self.brick_goal.position,gripper_poly_map):
                # The brick has left the gripper area - give up
                rospy.loginfo(rospy.get_name() + " Brick no longer in gripper area - giving up.")
                self.brick_goal = None
                self.status = STATUS_IDLE
                return
            # compute the required position adjustment, and send to motor controller
            #TODO: Simplify - can go directly from camera to gripper frame
            brick_map = PointStamped(header=Header(stamp=current_time, frame_id="map"), point=self.brick_goal.position)
            brick_base = self.listener.transformPoint("base_link",brick_map)
            if math.sqrt((brick_base.point.x - pose_gripper_base.position.x) ** 2 + (brick_base.point.y - pose_gripper_base.position.y) ** 2)  < GRIPPER_TOLERANCE:
                self.approaching_valid_pos_counter += 1
                rospy.loginfo(rospy.get_name() + " Identified {} frames at correct position ".format(self.approaching_valid_pos_counter))
                #wait for a few detections to be at the exact same base_link location - only then initiate gripper
                if self.approaching_valid_pos_counter > 5:
                    # Gripper is close enough - grip!
                    rospy.loginfo(rospy.get_name() + " Moving gripper forward")
                    self.gripper_proxy(action="MOVE", duration=GRIPPER_MOVE_TIME, position=GRIPPER_POS_DOWN)
                    self.wait_until = current_time + GRIPPER_MOVE_TIME
                    self.status = STATUS_GRIPPER_LOWERING
                else:
                    self.wait_until = current_time + Duration(0.2) #Wait a bit for next frame
            else:
                # Gripper is not close enough - adjust
                rospy.loginfo(rospy.get_name() + " Adjusting gripper position")
                self.approaching_valid_pos_counter=0
                #rospy.loginfo(rospy.get_name() + " base coordinates: brick: {}, gripper: {}".format(brick_base.point,pose_gripper_base.position))
                #rospy.loginfo(rospy.get_name() + " map coordinates: brick: {}, gripper: {}".format(brick_map.point,pose_gripper_map.position))
                forward_m = brick_base.point.x - pose_gripper_base.position.x #we simply take the difference in x, don't care for rotation (as rotation is small anyway)
                alpha_rad = math.atan2(brick_base.point.y, brick_base.point.x) - math.atan2(pose_gripper_base.position.y,pose_gripper_base.position.x)  # in rad
                delta_m= alpha_rad / 3.657 #Approximation. TODO: Clean up / merge with speed control one
                rospy.loginfo(rospy.get_name() + " Need to go forward {} m, with a delta of {} m".format(forward_m,delta_m))
                readpos_reply=self.readpos_proxy()
                forward_enc = enc_per_m * forward_m
                delta_enc = enc_per_m * delta_m
                positions_enc = [
                    readpos_reply.encoder[0] + forward_enc - delta_enc,
                    readpos_reply.encoder[1] + forward_enc - delta_enc,
                    readpos_reply.encoder[2] + forward_enc + delta_enc,
                    readpos_reply.encoder[3] + forward_enc + delta_enc
                ]
                time = Duration(3)  #TODO: Don't use constant, compute based on distance, plus a bit extra
                writeposentry = motorctrl_writeentry(time=readpos_reply.time + time, target=positions_enc)
                writeposentries = [writeposentry]
                self.writepos_proxy(writeposentries=writeposentries)
                self.wait_until = current_time + time
            return

        if self.status == STATUS_GRIPPER_LOWERING:
            rospy.loginfo(rospy.get_name() + " Closing gripper")
            self.gripper_proxy(action="CLOSE",duration=GRIPPER_GRIP_TIME)
            self.wait_until = current_time + GRIPPER_GRIP_TIME
            self.status = STATUS_GRIPPER_CLOSING
            return

        if self.status == STATUS_GRIPPER_CLOSING:
            rospy.loginfo(rospy.get_name() + " Moving gripper backward")
            self.gripper_proxy(action="MOVE", duration=GRIPPER_MOVE_TIME, position=GRIPPER_POS_UP)
            self.wait_until = current_time + GRIPPER_MOVE_TIME
            self.status = STATUS_GRIPPER_RAISING
            return

        if self.status == STATUS_GRIPPER_RAISING:
            rospy.loginfo(rospy.get_name() + " Opening gripper")
            self.gripper_proxy(action="OPEN",duration=GRIPPER_GRIP_TIME)
            self.wait_until = current_time + GRIPPER_GRIP_TIME
            self.status = STATUS_GRIPPER_OPENING
            return

        if self.status == STATUS_GRIPPER_OPENING:
            rospy.loginfo(rospy.get_name() + " Gripper opened")
            self.brick_goal = None
            self.status = STATUS_IDLE
            return

    def calc_distance(self,p1,p2):
        return math.sqrt((p1.position.x-p2.position.x) ** 2 + (p1.position.y-p2.position.y) ** 2)

    #Computes the base target pos required to approach a given brick
    def compute_base_target_pos(self,brick,pose_gripper_map,pose_gripper_base):
        # FUse current robot's orientation to attempt
        rospy.loginfo(rospy.get_name() + " gripper angle is {}".format(self.quaternion_to_angle(pose_gripper_map.orientation)))
        beta = 180.0 / math.pi * math.atan2(brick.position.y - pose_gripper_map.position.y,
                                            brick.position.x - pose_gripper_map.position.x)
        if beta < 0:
            beta = beta + 360
        orientation = self.angle_to_quaternion(beta)
        rospy.loginfo(rospy.get_name() + " orientation is {}".format(self.quaternion_to_angle(orientation)))
        #rospy.loginfo(rospy.get_name() + " Closest brick coordinates are {}".format(brick.position))
        #
        # TODO: Can I not use TF to do all these transformations?
        # alpha rad is negative if brick is to the right of the gripper, positive if brick is to the left of the gripper
        alpha_rad = math.pi / 180.0 * self.quaternion_to_angle(orientation)
        # compute gripper coordinates in map frame:
        g = pose_gripper_base.position.x + GRIPPER_DIST  # (x distance from base link to target position, i.e. sum of target distance from gripper plus distance base frame from gripper frame)
        # xg / yg are coordinates in map frame of the point which is on same x as base link, but at a different y (in base link frame)
        xg = brick.position.x - math.cos(alpha_rad) * g
        yg = brick.position.y - math.sin(alpha_rad) * g
        # compute base coordinates in map frame
        f = pose_gripper_base.position.y  # (y coordinate of gripper in base frame (negative value, as gripper is right of base link))
        # beta_rad=math.pi/2-alpha_rad
        xb = xg + f * math.sin(alpha_rad)
        yb = yg - f * math.cos(alpha_rad)
        return Pose(position=Point(x=xb, y=yb, z=0.0), orientation=orientation)


    def convert_poly_to_msg(self,poly):
        points=[]
        for p in poly:
            points.append(Point32(x=p.x,y=p.y,z=p.z))
        return Polygon(points=points)

    def callback(self,detectionarray):
        #rospy.loginfo(rospy.get_name() + " received detection for time stamp "+str(det.header.stamp.secs))

        #1. Calculate stable area(s) in map frame
        #2. Decrement all garbage collector counts for all bricks in visible area(s) If counter reaches zero, remove brick
        #3. For all detected bricks, compute cloud area
        #4. Find overlaps of detected cloud areas with existing detections - ideally find overlaps in a way that total distance is minimized across all matches
        #5. Update the overlapped bricks with new detection data, and reset the counter
        #6. Insert new bricks which had no overlap, with full counter
        #rospy.loginfo(rospy.get_name() + " waited for transform ")

        # Build the new bricks
        new_bricks=[]
        for i in range(0,len(detectionarray.detections)):
            singledetection = detectionarray.detections[i]
            p_cam=singledetection.position
            self.listener.waitForTransform("map", detectionarray.header.frame_id, singledetection.stamp, rospy.Duration.from_sec(10.0))
            p_map = self.listener.transformPoint("map", PointStamped(header=Header(frame_id=detectionarray.header.frame_id,stamp=singledetection.stamp),point=p_cam))
            radius=self.calc_radius(p_cam)
            new_bricks.append(Brick(position=p_map.point, color=self.convert_brick_color(singledetection.color), radius=radius, count=RESET_COUNT, source=singledetection.source, id=-1))

        # Remove double detections in near and far camera
        i=len(new_bricks) - 1
        while i>=0:
            if new_bricks[i].source=="FAR":
                #FAR detection found: if there is any NEAR detection within tolerance from this detection, remove FAR
                #This is to avoid placing duplicates in case the same brick appears in near and far detection
                for j in range(0,len(new_bricks)):
                    if detectionarray.detections[j].source=="NEAR" and self.calc_distance(new_bricks[i],new_bricks[j])<0.05:
                        del(new_bricks[i])
                        break
            i-=1

        #
        # Obtain lock to manipulate list of bricks
        stable_poly_map=self.transform_poly(camera_coord.STABLE_POLY, detectionarray.header.stamp, "camera_link", "map") #Theorteically, do this per brick with brick's timesetamps
        with self.lock:
            # Purges pool of visible bricks by decrementing the counters (only in stable detection area)
            i = len(self.pool) - 1
            while i >= 0:
                brick = self.pool[i]
                if self.ray_tracing(brick.position, stable_poly_map):
                    # Brick is in stable detection area - decrement the counter and remove if reaches zero
                    brick.dec_count()
                    if brick.count <= 0:
                        del self.pool[i]
                i = i - 1
            #
            # For each existing brick on entire map, try to find a newly detected brick which is in the radius.
            # If a new brick is found in the radius, it is assumed to be the same as the existing brick:
            # remove it from the list of newly detected bricks, and reset the counter in the existing brick.
            # at the end, what remains in the list of newly detected bricks
            #rospy.loginfo(rospy.get_name() + " detected {} new bricks, next id is {}".format(len(new_bricks),self.brick_id))
            for b in self.pool:
                i = len(new_bricks) - 1
                while i >= 0:
                    n = new_bricks[i]
                    distance = self.calc_distance(n,b)
                    #rospy.loginfo(rospy.get_name() + " Distance between new and {} is {}, sum of radius is {}".format(b.id,distance,n.radius+b.radius))
                    if distance < n.radius+b.radius:
                        #rospy.loginfo(rospy.get_name() + " new and {} overlap, removing new".format(b.id))
                        #Found an overlap of new with existing:
                        #1. Reset counter in existing, and update it's position etc
                        #2. remove the new one (as it was actually not a new one)
                        b.count=RESET_COUNT
                        if n.radius < b.radius:
                            #Update the position and the radius if the new detection is better (i.e. taken from closer)
                            b.radius=n.radius
                            b.position=n.position
                        del new_bricks[i]
                        break
                    i = i - 1
            #
            # Now assign an id to each new brick, as they are authentically new
            for n in new_bricks:
                n.id = self.create_new_brick_id()
                rospy.loginfo(rospy.get_name() + " Added new brick with id {}".format(n.id) )
            #
            # Put all new bricks into the pool
            self.pool.extend(new_bricks)
            #
            # Now send out an updated marker message with all current bricks
            header = Header(frame_id="map", stamp=detectionarray.header.stamp)
            markers=MarkerArray()
            markers.markers.append(Marker(action=Marker.DELETEALL,ns="brick"))
            markers.markers.append(Marker(action=Marker.DELETEALL,ns="cylinder"))
            markers.markers.append(Marker(action=Marker.DELETEALL,ns="text"))
            for i in range(0,len(self.pool)):
                brick=self.pool[i]
                # Create the cube
                pose=Pose(position=Point(x=brick.position.x, y=brick.position.y, z=brick.position.z + marker_scale.z / 2), orientation=forward)
                m=Marker(header=header,ns="brick",id=brick.id,type=Marker.CUBE,action=Marker.ADD,pose=pose,scale=marker_scale,color=brick.color)
                markers.markers.append(m)
                # Create the cylinder
                cyl_scale=Vector3(x=brick.radius*2,y=brick.radius*2,z=marker_scale.z/2)
                cyl_m=Marker(header=header,ns="cylinder",id=brick.id,type=Marker.CYLINDER,action=Marker.ADD,pose=pose,scale=cyl_scale,color=cyl_color)
                markers.markers.append(cyl_m)

                # Create the label
                text_point=Point(x=brick.position.x, y=brick.position.y, z=brick.position.z + 0.1)
                test_pose=Pose(position=text_point,orientation=forward)
                text_m=Marker(header=header,ns="text",id=brick.id,type=Marker.TEXT_VIEW_FACING,action=Marker.ADD,pose=test_pose,scale=text_scale,color=text_color,text=str(brick.id))
                markers.markers.append(text_m)
        #rospy.loginfo(rospy.get_name() + " publishing markers ")
        self.markers_pub.publish(markers)
        #rospy.loginfo(rospy.get_name() + " markers published")

    def create_new_brick_id(self):
        id = self.brick_id
        self.brick_id =self.brick_id+1
        return id

    #The point must be in camera frame. Find the radius of uncertainty
    def calc_radius(self,point):
        #TODO: for near, use a const, for far, use some sophisticated formula
        distance=math.sqrt(point.x**2+point.y**2)
        if distance < 0.12:
            # This is near camera: return const radius size of brick:
            return 0.02
        else:
            # In far vision, apply linear scaling
            x=max(0,distance-0.12)
            radius=0.04 + x*0.36  #At 1meter, radius is 0.04 + 0.36 = 0.40 m
            return radius

    def transform_poly(self,poly,stamp,frame_from,frame_to):
        poly_tf=[]
        for p in poly:
            ps=PointStamped(header=Header(stamp=stamp,frame_id=frame_from),point=p)
            p_tf = self.listener.transformPoint(frame_to, ps)
            poly_tf.append(p_tf.point)
        return poly_tf

    #Converts the natural brick color in the best matching intense color
    def convert_brick_color(self,color):
        best_match = None
        best_diff = 0
        for c in BRICK_COLORS:
            s=((c.r+c.g+c.b)/(color.r+color.g+color.b+0.0001)) #add to avoid div 0
            color_diff = abs(c.r-color.r*s)+abs(c.g-color.g*s)+abs(c.b-color.b*s)
            if best_match is None or color_diff<best_diff:
                best_match = c
                best_diff = color_diff
        return best_match



    def ray_tracing(self,point,poly):
        n = len(poly)
        inside = False
        xints = 0.0
        p1= poly[0]
        for i in range(n+1):
            p2 = poly[i % n]
            if point.y > min(p1.y,p2.y):
                if point.y <= max(p1.y,p2.y):
                    if point.x <= max(p1.x,p2.x):
                        if p1.y != p2.y:
                            xints = (point.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x
                        if p1.x == p2.x or point.x <= xints:
                            inside = not inside
            p1=p2
        return inside

    # returns the angle between o2 and o1, where 0<=angle<=360
    def quaternion_to_angle(self,q):
        angle = 2 * math.acos(q.w) * 180 / math.pi
        if q.z < 0:
            angle = 360 - angle
        return angle

    # returns a quaternion, 0<=angle<=360
    def angle_to_quaternion(self,angle):
        w = math.cos(angle * math.pi / 2 / 180)
        z = math.sqrt(1-w**2)
        if angle > 180:
            w = -w
            z = -z
        return Quaternion(w=w,x=0,y=0,z=z)

# Main function.
if __name__ == '__main__':
    node = Node()

