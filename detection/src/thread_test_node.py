#!/usr/bin/env python

import threading
import rospy
from time import sleep


class Node:
    # Power on and prepare for general usage.
    # This will activate the device and take it out of sleep mode (which must be done
    # after start-up). This function also sets both the accelerometer and the gyroscope
    # to default range settings, namely +/- 2g and +/- 250 degrees/sec.
    def __init__(self):
        rospy.init_node('thread_test_node')
        interval = rospy.Duration.from_sec(0.01)
        rospy.Timer(interval, self.callback_tim1)
        rospy.Timer(interval, self.callback_tim2)
        rospy.spin()

    def callback_tim1(self,timer_event):
        sleep(2)
        rospy.loginfo(rospy.get_name() + " tim1 {}".format(threading.current_thread().ident))

    def callback_tim2(self,timer_event):
        sleep(3)
        rospy.loginfo(rospy.get_name() + " tim2 {}".format(threading.current_thread().ident))

# Main function.
if __name__ == '__main__':
    node = Node()

