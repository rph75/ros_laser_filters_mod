#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import PoseStamped, Twist, Vector3, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry


def quaternion_to_rad(q):
    rad=2*math.acos(q.w)
    if q.z<0:
        rad=2*math.pi-rad
    return rad

def rad_to_deg(rad):
    return 180/math.pi*rad

def callback(pose_stamped):
    global odom_pub
    global last_pose_stamped
    if last_pose_stamped is not None:
        secs = (pose_stamped.header.stamp - last_pose_stamped.header.stamp).to_sec()
        rospy.loginfo(rospy.get_name() + " duration "+str(secs))
        x1 = last_pose_stamped.pose.position.x
        y1 = last_pose_stamped.pose.position.y
        x2 = pose_stamped.pose.position.x
        y2 = pose_stamped.pose.position.y
        distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        pose_rad = quaternion_to_rad(pose_stamped.pose.orientation)
        last_pose_rad = quaternion_to_rad(last_pose_stamped.pose.orientation)
        rad = pose_rad - last_pose_rad
        if rad > math.pi:
            #Modulo 2 pi
            rad = rad - 2 * math.pi
        elif rad < -math.pi:
            #Modulo 2 pi
            rad = 2 * math.pi + rad

        rospy.loginfo(rospy.get_name() + " angle 1 "+str(rad_to_deg(last_pose_rad))+" angle 2 "+str(rad_to_deg(pose_rad))+" delta angle "+str(rad_to_deg(rad)))

        speed_rad = rad / secs
        #Find out if robot moved forward or backward:
        #If the angle between the two vectors is 'similar' to the angle of the original vector, then it is forward
        if abs(x2-x1)<0.000001: #handle div 0 (with epsilon)
            if y2>y1:
                alpha = math.pi/2 #90 degs
            else:
                alpha = -math.pi / 2  # -90 degs
        else:
            alpha = math.atan((y2-y1)/(x2-x1))
            rospy.loginfo(rospy.get_name() + " alpha " + str(rad_to_deg(alpha)))
            if x2 < x1:
                #Adjust for other direction
                alpha = math.pi + alpha
        gamma = abs(last_pose_rad - alpha)
        rospy.loginfo(rospy.get_name() + " alpha "+str(rad_to_deg(alpha))+", gamma " + str(rad_to_deg(gamma)))
        if gamma > math.pi:
            #Modulo 2 pi
            gamma = 2 * math.pi - gamma
        if gamma > math.pi / 2:
            #vector last pose and vector from last pose to new pose point in different direction -> robot is going backward
            distance = -distance
        speed = distance / secs
        linear = Vector3(x=speed,y=0,z=0)
        angular = Vector3(x=0,y=0,z=speed_rad)
        twist = Twist(linear=linear,angular=angular)
        rospy.loginfo(rospy.get_name() + " speed "+str(speed)+" twist "+str(speed_rad))
        no_covariance=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        pose_with_cov = PoseWithCovariance(pose=pose_stamped.pose,covariance=no_covariance)
        twist_with_cov = TwistWithCovariance(twist=twist,covariance=no_covariance)
        odometry = Odometry(header = pose_stamped.header, child_frame_id = "base_link", pose = pose_with_cov, twist = twist_with_cov)
        odom_pub.publish(odometry)
    last_pose_stamped = pose_stamped

# Main function.
if __name__ == '__main__':
    global odom_pub
    global last_pose_stamped
    last_pose_stamped = None
    # Initialize the node and name it.
    rospy.init_node('pose_to_odom')
    rospy.Subscriber("slam_out_pose", PoseStamped, callback)
    rospy.loginfo(rospy.get_name() + " subscribed to slam_out_pose.")
    odom_pub = rospy.Publisher("odom", Odometry)

    rospy.spin()


