#!/usr/bin/env python

import numpy as np
#import cv2 #Must be imported before importing tensorflow, otherwise some strange error god knows why
import threading
import rospy
import math
import tf
import operator
import coordinates
from detection.msg import detection
from std_msgs.msg import Header,ColorRGBA
from geometry_msgs.msg import PointStamped, Point, PoseArray, Pose, PoseStamped, Quaternion, Twist, Vector3, Polygon, PolygonStamped, Point32
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal, MoveBaseAction
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

# Camera ares, relative to camera link
NEAR_POLY = [Point(x=0.18, y=-0.18), Point(x=0.08, y=0.02), Point(x=0.01, y=-0.01), Point(x=0.01, y=-0.14), Point(x=0.05, y=-0.17)]
#TODO:
far_poly = [coordinates.to_base_link_point(17, 17), coordinates.to_base_link_point(71,155), coordinates.to_base_link_point(-38,155), coordinates.to_base_link_point(2,17)]

def add_to_point(point,offset):
    return Point(x=point.x+offset[0],y=point.y+offset[1],z=point.z+offset[1])

# Camera areas, slightly reduced in size to have a tolerance area where bricks are not garbage collected if they do not appear
OFFSET_NEAR = 0.01 # offset of smaller polygon, in meters
near_poly_small = [
    add_to_point(NEAR_POLY[0], (OFFSET_NEAR, OFFSET_NEAR , 0)),
    add_to_point(NEAR_POLY[1], (-OFFSET_NEAR, OFFSET_NEAR , 0)),
    add_to_point(NEAR_POLY[2], (-OFFSET_NEAR, -OFFSET_NEAR , 0)),
    add_to_point(NEAR_POLY[3], (OFFSET_NEAR, -OFFSET_NEAR , 0)),
    TODO
]
OFFSET_FAR1 = 0.02 # offset of smaller polygon, for the two points closer to the camrea, in meters
OFFSET_FAR2 = 0.10 # offset of smaller polygon, for the two points further from the camrea, in meters
far_poly_small = [
    add_to_point( far_poly[0], (OFFSET_FAR1, OFFSET_FAR1 ,0)),
    add_to_point( far_poly[1], (-OFFSET_FAR2, OFFSET_FAR2 ,0)),
    add_to_point( far_poly[2], (-OFFSET_FAR2, -OFFSET_FAR2 ,0)),
    add_to_point( far_poly[3], (OFFSET_FAR1, -OFFSET_FAR1 ,0)),
]


# Gripper area, relative to gripper coordinate frame
gripper_poly = [Point(x=-0.05,y=-0.11,z=0.0), Point(x=0.20,y=-0.11,z=0.0),Point(x=0.20,y=0.11,z=0.0),Point(x=-0.05,y=0.11,z=0.0)]

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

ATTEMPT_ANGLE = 90      #If a brick is not reachable from a specific angle, try to reach from a different angle
GRIPPER_DIST = 0.15     #Distance from the brick when setting a goal

MAX_GRIP_ATTEMPTS = 2   #Number of grip attempts until we're giving up gripping an object and set it to unreachable

GRIPPER_TOLERANCE = 0.003  #Max distance in mm from gripper in order to pick up the piece

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
    def __init__(self,position,color,radius,count,id=-1):
        self.id=id
        self.position=position
        self.color=color
        self.radius=radius
        self.count=count
        self.unreachable=False
        self.attempted_orientations=[]
        self.grip_attempts = 0 #Number of times we've tried to grip this object

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
        rospy.Subscriber("detection", detection, self.callback,queue_size=2) #TODO: Cannot process fast enough, therefore dropping - is this good?
        self.markers_pub = rospy.Publisher("bricks", MarkerArray, queue_size=10)
        self.camera_area_near_pub = rospy.Publisher("camera_area_near", PolygonStamped, queue_size=10)
        self.camera_area_far_pub = rospy.Publisher("camera_area_far", PolygonStamped, queue_size=10)
        self.camera_area_near_small_pub = rospy.Publisher("camera_area_near_small", PolygonStamped, queue_size=10)
        self.camera_area_far_small_pub = rospy.Publisher("camera_area_far_small", PolygonStamped, queue_size=10)
        self.gripper_area_pub = rospy.Publisher("gripper_area", PolygonStamped, queue_size=10)
        self.idle_waypoints_pub = rospy.Publisher("idle_waypoints", PoseArray, queue_size=10)
        interval = rospy.Duration.from_sec(0.5)
        rospy.Timer(interval, self.callback_tim)
        self.move_base_action.cancel_goal() #Cancel any previous goal, such that when this node gets started we are clean
        rospy.loginfo(rospy.get_name() + " started bricks_tracker_node.")
        rospy.spin()

    def callback_tim(self,timer_event):
        ps_near=PolygonStamped(header=Header(frame_id="camera_link"), polygon=self.convert_poly_to_msg(NEAR_POLY))
        self.camera_area_near_pub.publish(ps_near)
        ps_far=PolygonStamped(header=Header(frame_id="camera_link"),polygon=self.convert_poly_to_msg(far_poly))
        self.camera_area_far_pub.publish(ps_far)

        ps_near_small=PolygonStamped(header=Header(frame_id="camera_link"),polygon=self.convert_poly_to_msg(near_poly_small))
        self.camera_area_near_small_pub.publish(ps_near_small)
        ps_far_small=PolygonStamped(header=Header(frame_id="camera_link"),polygon=self.convert_poly_to_msg(far_poly_small))
        self.camera_area_far_small_pub.publish(ps_far_small)

        ps_gripper=PolygonStamped(header=Header(frame_id="gripper"),polygon=self.convert_poly_to_msg(gripper_poly))
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
        gripper_poly_map = self.transform_poly(gripper_poly, current_time, "gripper", "map")
        if self.status in [STATUS_IDLE,STATUS_APPROACHING,STATUS_MOVINGBASE]:
            # Check if the brick still exists and is still reachable - otherwise cancel goal (if there is any) and go back to idle status
            with self.lock:
                if self.brick_goal is not None and (self.brick_goal not in self.pool or self.brick_goal.unreachable):
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
                    if not b.unreachable:
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
                    self.status = STATUS_APPROACHING
                    return
        #
        if self.status == STATUS_IDLE:
            with self.lock:
                #Find the closest brick (if any) - and move towards it
                closest_brick = None
                for b in self.pool:
                    if not b.unreachable:
                        d = self.calc_distance(b, pose_gripper_map);
                        if closest_brick is None or d < distance:
                            distance = d
                            closest_brick = b
            # check if we found a closest brick
            if closest_brick is not None:
                rospy.loginfo(rospy.get_name() + " Closest brick is {}, sending goal".format(closest_brick.id))
                #self.next_idle_waypoint = None # reset idle path
                self.brick_goal = closest_brick
                self.move_base_to_brick(current_time,pose_gripper_map,pose_gripper_base)
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
                d = self.calc_distance(self.brick_goal,pose_gripper_map);
                if d<self.goal_set_at_distance*0.7:
                    rospy.loginfo(rospy.get_name() + " Readjusting goal because I am getting closer (now at {} m)".format(d))
                    self.move_base_to_brick(current_time, pose_gripper_map, pose_gripper_base)
                return
            if goal_status != GoalStatus.SUCCEEDED:
                # Finished, but with error - mark this as unreachable orientation
                rospy.loginfo(rospy.get_name() + " Could not reach goal")
                self.brick_goal.attempted_orientations.append(self.target_pose.orientation)
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
            brick_map = PointStamped(header=Header(stamp=current_time, frame_id="map"), point=self.brick_goal.position)
            brick_base = self.listener.transformPoint("base_link",brick_map)
            if math.sqrt((pose_gripper_base.position.x-brick_base.point.x)**2+(pose_gripper_base.position.y-brick_base.point.y)**2) < GRIPPER_TOLERANCE:
                #TODO: wait for a few detections to be at the exact same base_link location - only then initiate gripper
                # Gripper is close enough - grip!
                rospy.loginfo(rospy.get_name() + " Moving gripper forward")
                self.gripper_proxy(action="MOVE", duration=GRIPPER_MOVE_TIME, position=GRIPPER_POS_DOWN)
                self.wait_until = current_time + GRIPPER_MOVE_TIME
                self.status = STATUS_GRIPPER_LOWERING
            else:
                # Gripper is not close enough - adjust
                rospy.loginfo(rospy.get_name() + " Adjusting gripper position")
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
            self.brick_goal.grip_attempts = self.brick_goal.grip_attempts +1
            if self.brick_goal.grip_attempts > MAX_GRIP_ATTEMPTS:
                #Tried too often to grip this - set to unreachable
                self.brick_goal.unreachable = True
            self.brick_goal = None
            self.status = STATUS_IDLE
            return

    def calc_distance(self,p1,p2):
        return math.sqrt((p1.position.x-p2.position.x) ** 2 + (p1.position.y-p2.position.y) ** 2)


    def move_base_to_brick(self,current_time,pose_gripper_map,pose_gripper_base):
        # Find an orientation which was not yet attempted, preferrably the current robot's orientation, otherwise
        rospy.loginfo(rospy.get_name() + " gripper angle is {}".format(self.quaternion_to_angle(pose_gripper_map.orientation)))
        beta = 180.0 / math.pi * math.atan2(self.brick_goal.position.y - pose_gripper_map.position.y,
                                            self.brick_goal.position.x - pose_gripper_map.position.x)
        if beta < 0:
            beta = beta + 360
        best_orientation = self.angle_to_quaternion(beta)
        rospy.loginfo(rospy.get_name() + " best_orientation is {}".format(self.quaternion_to_angle(best_orientation)))
        orientation = self.find_ok_orientation(best_orientation, self.brick_goal.attempted_orientations)
        #rospy.loginfo(rospy.get_name() + " Closest brick coordinates are {}".format(self.brick_goal.position))
        if orientation is None:
            # Mark this object as non reachable
            rospy.loginfo(rospy.get_name() + " Closest brick is not reachable. Giving up.")
            self.brick_goal.unreachable = True
            return  # Return, keep status unchanged. Next timer will attempt next brick
        rospy.loginfo(rospy.get_name() + " ok orientation for brick {} is {} ".format(self.brick_goal.id,self.quaternion_to_angle(orientation)))
        #
        # TODO: Can I not use TF to do all these transformations?
        # alpha rad is negative if brick is to the right of the gripper, positive if brick is to the left of the gripper
        alpha_rad = math.pi / 180.0 * self.quaternion_to_angle(orientation)
        # compute gripper coordinates in map frame:
        g = pose_gripper_base.position.x + GRIPPER_DIST  # (x distance from base link to target position, i.e. sum of target distance from gripper plus distance base frame from gripper frame)
        # xg / yg are coordinates in map frame of the point which is on same x as base link, but at a different y (in base link frame)
        xg = self.brick_goal.position.x - math.cos(alpha_rad) * g
        yg = self.brick_goal.position.y - math.sin(alpha_rad) * g
        # compute base coordinates in map frame
        f = pose_gripper_base.position.y  # (y coordinate of gripper in base frame (negative value, as gripper is right of base link))
        # beta_rad=math.pi/2-alpha_rad
        xb = xg + f * math.sin(alpha_rad)
        yb = yg - f * math.cos(alpha_rad)
        target_pose_base_map = Pose(position=Point(x=xb, y=yb, z=0.0), orientation=orientation)
        #rospy.loginfo(rospy.get_name() + " Going with base link to coordinates {}".format(target_pose_base_map))
        rospy.loginfo(rospy.get_name() + " Sending goal to move base")
        self.status = STATUS_MOVINGBASE
        self.target_pose = target_pose_base_map
        #Update the distance to the brick when goal was last set
        self.goal_set_at_distance = self.calc_distance(self.brick_goal,pose_gripper_map);
        #Send the action
        self.move_base_action.send_goal(MoveBaseGoal(target_pose=PoseStamped(header=Header(frame_id="map", stamp=current_time), pose=self.target_pose)))
        return

    def convert_poly_to_msg(self,poly):
        points=[]
        for p in poly:
            points.append(Point32(x=p.x,y=p.y,z=p.z))
        return Polygon(points=points)

    def callback(self,det):
        #rospy.loginfo(rospy.get_name() + " received detection for time stamp "+str(det.header.stamp.secs))

        #1. Calculate visible area(s) in map frame
        #2. Decrement all garbage collector counts for all bricks in visible area(s) If counter reaches zero, remove brick
        #3. For all detected bricks, compute cloud area
        #4. Find overlaps of detected cloud areas with existing detections - find overlaps in a way that total distance is minimized across all matches
        #5. Update the overlapped bricks with new detection data, and reset the counter
        #6. Insert new bricks which had no overlap, with full counter
        self.listener.waitForTransform("map",det.header.frame_id, det.header.stamp, rospy.Duration.from_sec(10.0))
        #rospy.loginfo(rospy.get_name() + " waited for transform ")
        near_poly_map=self.transform_poly(NEAR_POLY, det.header.stamp, "camera_link", "map")
        far_poly_map=self.transform_poly(far_poly,det.header.stamp,"camera_link", "map")
        near_poly_small_map=self.transform_poly(near_poly_small,det.header.stamp,"camera_link","map")
        far_poly_small_map=self.transform_poly(far_poly_small,det.header.stamp,"camera_link", "map")
        new_bricks=[]
        for i in range(0,len(det.detections)):
            p_base=det.detections[i]
            p_map = self.listener.transformPoint("map", PointStamped(header=det.header,point=p_base))
            radius=self.calc_radius(p_base)
            if self.is_visible(p_map.point, [near_poly_map, far_poly_map]):
                #Append new bricks only if within visible range - otherwise we might put bricks outside the visible range which would
                #then not be removed (if the poly is smaller than the actually detected range)
                new_bricks.append(Brick(position=p_map.point, color=self.convert_brick_color(det.colors[i]), radius=radius, count=RESET_COUNT))
            #else:
                #rospy.loginfo(rospy.get_name() + " detected point which is outside the visible range: base_link: "+str(p_base))
        #
        # Obtain lock to manipulate list of bricks
        #rospy.loginfo(rospy.get_name() + " obtaining lock ")
        with self.lock:
            #rospy.loginfo(rospy.get_name() + " lock obtained ")
            # Purge visible bricks (in reduced area, to not purge stuff that's just on the border)
            self.purge_visible_pool([near_poly_small_map,far_poly_small_map])
            #
            # For each existing brick, try to find a newly detected brick which is in the radius.
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
                            #Update the position and the radius only if the new detection is better (i.e. taken from closer)
                            b.radius=n.radius
                            b.position=n.position
                        del new_bricks[i]
                        break
                    i = i - 1
            #
            # Now assign a id to each new brick, as they are authentically new
            for n in new_bricks:
                n.id = self.create_new_brick_id()
                rospy.loginfo(rospy.get_name() + " Added new brick with id {}".format(n.id) )
            #
            # Put all new bricks into the pool
            self.pool.extend(new_bricks)
            #
            # Now send out an updated marker message with all current bricks
            header = Header(frame_id="map", stamp=det.header.stamp)
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

    #The point must be in base_link frame. Find the radius of uncertainty
    def calc_radius(self,point):
        #TODO: for near, use a const, for far, use some sophisticated formula
        distance=math.sqrt(point.x**2+point.y**2)
        if distance < 0.31:
            # This is near vision: return const radius size of brick:
            return 0.02
        else:
            # In far vision, apply linear scaling
            x=max(0,distance-0.31)
            radius=0.02 + x*0.04
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

    #Purges pool of visible bricks by decrementing the counters
    def purge_visible_pool(self,poly_maps):
        i=len(self.pool)-1
        while i>=0:
            brick=self.pool[i]
            if self.is_visible(brick.position, poly_maps):
                #Brick is visible - decrement the counter and remove if reaches zero
                brick.dec_count()
                if brick.count <= 0:
                    del self.pool[i]
            i=i-1

    def is_visible(self,point,poly_maps):
        for poly_map in poly_maps:
            if self.ray_tracing(point,poly_map):
                return True
        return False


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

    # Tries to find an unattempted orientation from where we could attack the brick
    # orientation: preferred orientation
    # attempted_orientations: already unsuccessfully attempted orientations
    def find_ok_orientation(self,orientation,attempted_orientations):
        if self.is_orientation_ok(orientation,attempted_orientations):
            #The provided orientation is ok
            return orientation
        rospy.loginfo(rospy.get_name() + " initially attempted orientation {} is not ok".format(self.quaternion_to_angle(orientation)))

        #
        # Try to find an orientation which is acceptable, and is closest to current orientation
        closest_orientation = None
        best_angle = -1
        for o in attempted_orientations:
            angle = self.angle(orientation,o)
            if closest_orientation is None or abs(angle)<abs(best_angle):
                closest_orientation=o
                best_angle=angle
        #Try left and right of the closest orientation until we find a matching one
        angle = ATTEMPT_ANGLE
        while abs(angle) < 180:
            o = self.add_angle(closest_orientation,angle)
            rospy.loginfo(rospy.get_name() + " attempting orientation {} ".format(self.quaternion_to_angle(o)))
            if self.is_orientation_ok(o,attempted_orientations):
                rospy.loginfo(rospy.get_name() + " orientation {} is ok".format(self.quaternion_to_angle(o)))
                return o
            if angle < 0:
                angle = -angle+ATTEMPT_ANGLE
            else:
                angle = -angle
        #Nothing found
        return None

    # Returns the angle between o2 and o1, where -180<=angle<=180
    # positive value of o2 is in counterclockwise direction of o1
    def angle(self,o1,o2):
        angle1 = self.quaternion_to_angle(o1)
        angle2 = self.quaternion_to_angle(o2)
        diff = angle2-angle1
        if diff<-180:
            diff=diff+360
        elif diff>180:
            diff=diff-360
        return diff

    # positive angle will rotate orientation counterclockwise
    def add_angle(self,orientation,angle):
        a=self.quaternion_to_angle(orientation) #0 to 360, counterclockwise
        a=a+angle
        while a>360:
            a=a-360
        return self.angle_to_quaternion(a)


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


    def is_orientation_ok(self,orientation,attempted_orientations):
        for o in attempted_orientations:
            if abs(self.angle(orientation,o)) < ATTEMPT_ANGLE:
                return False
        return True

# Main function.
if __name__ == '__main__':
    node = Node()

