#!/usr/bin/env python

#This node is used to calibrate odometry of the robot
#It will perform a couple of movements, using different speed configurations (patterns)
#And measure how far it got. This data is then saved to a CSV file which can later be used to process
#And calibrate the odometry

import rospy,math
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

from motorctrl.srv import motorctrl_readpos, motorctrl_readspeed, motorctrl_writepos, motorctrl_writespeed
from motorctrl.msg import motorctrl_writeentry

ENC_PER_M=11500 #encoder units per meter
ENC_PER_RAD=3000 #encoder units per rad turn. Good guess?

def add_milli_seconds(time,msecs):
    add_secs=int(msecs/1000)
    add_nsecs=(msecs-(add_secs*1000))*1000000
    return rospy.Time(secs=time.secs+add_secs,nsecs=time.nsecs+add_nsecs)

def callback_odometry(odometry):
    return

def callback_pose(pose):
    global write_first_pose_at
    global write_last_pose_at
    global first_pose
    global last_pose
    if (pose.header.stamp > write_first_pose_at) & (first_pose is None):
        first_pose = pose
    elif pose.header.stamp < write_last_pose_at:
        last_pose = pose
    return


# tuples of speed (m/s), speed delta (m/s), duration (ms)
patterns=[
    #Linear movement:
    # (0.1, 0, 20000),
    # (-0.1, 0, 20000),
    # (0.2, 0, 10000),
    # (-0.2, 0, 10000),
    # (0.3, 0, 6000),
    # (-0.3, 0, 6000),
    # (0.4, 0, 5000),
    # (-0.4, 0, 5000),
    # (0.5, 0, 4000),
    # (-0.5, 0, 4000),
    #In place rotations:
    # (0.0, 0.05, 10000),
    # (0.0, -0.05, 10000),
    # (0.0, 0.075, 5000),
    # (0.0, -0.075, 5000),
    #Turns
    # (0.1, 0.01, 10000),
    # (-0.1, -0.01, 10000),
    # (0.1, 0.02, 10000),
    # (-0.1, -0.02, 10000),
    # (0.1, 0.03, 10000),
    # (-0.1, -0.03, 10000),
    # (0.1, 0.04, 10000),
    # (-0.1, -0.04, 10000),
    # (0.1, 0.05, 10000),
    # (-0.1, -0.05, 10000),
    # (0.1, 0.06, 10000),
    # (-0.1, -0.06, 10000),
    # (0.1, 0.07, 10000),
    # (-0.1, -0.07, 10000),
    # (0.1, 0.08, 10000),
    # (-0.1, -0.08, 10000),
    # (0.1, 0.09, 10000),
    # (-0.1, -0.09, 10000),
    # (0.1, 0.1, 10000),
    # (-0.1, -0.1, 10000),
    #
    # (0.1, 0.002, 10000),
    # (-0.1, -0.002, 10000),
    # (0.1, 0.004, 10000),
    # (-0.1, -0.004, 10000),
    # (0.1, 0.006, 10000),
    # (-0.1, -0.006, 10000),
    # (0.1, 0.008, 10000),
    # (-0.1, -0.008, 10000),

    # (0.2, 0.004, 10000),
    # (-0.2, -0.004, 10000),
    # (0.2, 0.008, 10000),
    # (-0.2, -0.008, 10000),
    # (0.2, 0.012, 10000),
    # (-0.2, -0.012, 10000),
    # (0.2, 0.016, 10000),
    # (-0.2, -0.016, 10000),
    # (0.2, 0.02, 10000),
    # (-0.2, -0.02, 10000),
    # (0.2, 0.04, 10000),
    # (-0.2, -0.04, 10000),
    # (0.2, 0.06, 10000),
    # (-0.2, -0.06, 10000),
    # (0.2, 0.08, 10000),
    # (-0.2, -0.08, 10000),

  (-0.18, 0.06, 8000),

    # (0.06, 0.01, 20000), (-0.06, -0.01, 20000),
    # (0.06, 0.03, 20000), (-0.06, -0.03, 20000),
    # (0.06, 0.05, 10000), (-0.06, -0.05, 10000),
    # (0.06, 0.07, 10000), (-0.06, -0.07, 10000),
    # (0.09, 0.01, 15000), (-0.09, -0.01, 15000),
    # (0.09, 0.03, 15000), (-0.09, -0.03, 15000),
    # (0.09, 0.05, 15000), (-0.09, -0.05, 15000),
    # (0.09, 0.07, 10000), (-0.09, -0.07, 10000),
    # (0.12, 0.01, 10000), (-0.12, -0.01, 10000),
    # (0.12, 0.03, 10000), (-0.12, -0.03, 10000),
    # (0.12, 0.05, 10000), (-0.12, -0.05, 10000),
    # (0.12, 0.07, 10000), (-0.12, -0.07, 10000),
    # (0.15, 0.01, 10000), (-0.15, -0.01, 10000),
    # (0.15, 0.03, 10000), (-0.15, -0.03, 10000),
    # (0.15, 0.05, 10000), (-0.15, -0.05, 10000),
    # (0.15, 0.07, 10000), (-0.15, -0.07, 10000),

]

def quaternion_to_angle(q):
    angle=2*math.acos(q.w)*180/math.pi
    if q.z<0:
        angle=360-angle
    return angle

def callback_tim(timer_event):
    global file
    global next_event_at
    global pattern_idx
    global write_first_pose_at
    global write_last_pose_at
    global first_pose
    global last_pose
    if pattern_idx > len(patterns):
        # No more
        return
    if rospy.Time().now() > next_event_at:
        print("Next event at")
        if pattern_idx > 0:
            time_delta=last_pose.header.stamp-first_pose.header.stamp
            angle_first=quaternion_to_angle(first_pose.pose.orientation)
            angle_last=quaternion_to_angle(last_pose.pose.orientation)
            line = str(patterns[pattern_idx-1][0]) +"," + str(patterns[pattern_idx-1][1]) +"," + str(time_delta) +"," + str(first_pose.pose.position_map.x) + "," + str(first_pose.pose.position_map.y) + "," + str(angle_first) + "," + str(last_pose.pose.position_map.x) + "," + str(last_pose.pose.position_map.y) + "," + str(angle_last) + "\n"
            print(line)
            file.write(line)
            print("first_pose "+str(first_pose))
            print("last_pose "+str(last_pose))
            if pattern_idx == len(patterns):
                #This was the last pose
                file.close()
                pattern_idx = pattern_idx + 1
                return
        pattern = patterns[pattern_idx]
        pattern_idx = pattern_idx + 1
        now = rospy.Time.now()
        time = add_milli_seconds(now,pattern[2])
        enc_per_sec = ENC_PER_M * pattern[0]
        enc_adjust_per_sec = ENC_PER_M * pattern[1]
        enc_per_sec_left = enc_per_sec - enc_adjust_per_sec
        enc_per_sec_right = enc_per_sec + enc_adjust_per_sec
        speeds = [
            enc_per_sec_left,
            enc_per_sec_left,
            enc_per_sec_right,
            enc_per_sec_right
        ]
        write_first_pose_at = add_milli_seconds(now,500)
        write_last_pose_at = add_milli_seconds(now,pattern[2]-300)
        next_event_at = add_milli_seconds(now,pattern[2]+2000)
        first_pose = None
        last_pose = None
        writespeed_proxy = rospy.ServiceProxy('motorctrl_service/writespeed', motorctrl_writespeed)
        writespeedentry = motorctrl_writeentry(time=time, target=speeds)
        writespeedentries = [writespeedentry]
        writespeed_response = writespeed_proxy(writespeedentries=writespeedentries)


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    global pattern_idx
    global next_event_at
    global file
    file = open("/home/roman/pos_measure.txt", "a+")
    rospy.init_node('speed_ctrl')
    next_event_at = rospy.Time().now()

    rospy.loginfo(rospy.get_name() + " Waiting for service /motorctrl_service/* ...")
    rospy.wait_for_service('/motorctrl_service/readpos')
    rospy.wait_for_service('/motorctrl_service/readspeed')
    rospy.wait_for_service('/motorctrl_service/writepos')
    rospy.wait_for_service('/motorctrl_service/writespeed')
    rospy.loginfo(rospy.get_name() + " Found service /motorctrl_service/* .")

    rospy.Subscriber("slam_out_pose", PoseStamped, callback_pose)
    rospy.Subscriber("odom", Odometry, callback_odometry)
    period_secs=0.020
    pattern_idx = 0
    interval = rospy.Duration.from_sec(period_secs)
    rospy.Timer(interval,callback_tim)
    rospy.loginfo(rospy.get_name() + " subscribed to cmd_vel.")
    rospy.spin()

