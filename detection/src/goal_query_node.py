#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal, MoveBaseAction
import actionlib

status=[
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

class Node:
    # Power on and prepare for general usage.
    # This will activate the device and take it out of sleep mode (which must be done
    # after start-up). This function also sets both the accelerometer and the gyroscope
    # to default range settings, namely +/- 2g and +/- 250 degrees/sec.
    def __init__(self):
        rospy.init_node('goal_query_node')
        self.move_base_action = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        interval = rospy.Duration.from_sec(1.5)
        rospy.Timer(interval, self.callback_tim)
        self.move_base_action.cancel_goal() #Cancel any previous goal, such that when this node gets started we are clean
        rospy.loginfo(rospy.get_name() + " started goal_query_node.")
        rospy.spin()

    def callback_tim(self,timer_event):
        # Now trace the stuff
        goal_status = self.move_base_action.get_state()
        rospy.loginfo(rospy.get_name() + " goal_status is {}, {} ".format(goal_status,status[goal_status]))


# Main function.
if __name__ == '__main__':
    node = Node()

