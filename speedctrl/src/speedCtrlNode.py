#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist
from motorctrl.srv import motorctrl_readpos, motorctrl_readspeed, motorctrl_writepos, motorctrl_writespeed
from motorctrl.msg import motorctrl_writeentry

#TODO: Move to params file

# Duration during which the robot will move, on receipt of a Twist message on cmd_vel.
# This includes the acceleration, the peak, and the deceleration phases
# For continuous movement, this should be a bit longer than the publish interval to the cmd_vel topic
enc_per_m=11500.0 #encoder units per cm #TODO: centralize this value (shared with odom_data)
INTERVAL_SECS = 2
MAX_ANG_SPEED=0.26
MAX_SPEED=0.15
#ENC_PER_RAD=3000 #encoder units per rad turn. Good guess?
#ACCELERATION=0.1 #in m per square second
#NUM_ACCELERATION_STEPS=5 #Number of steps to compute acceleration (1= one single step, no smoothing)

class Node:
    def __init__(self):
        rospy.init_node('speed_ctrl')
        rospy.loginfo(rospy.get_name() + " Waiting for service /motorctrl_service/* ...")
        rospy.wait_for_service('/motorctrl_service/readpos')
        rospy.wait_for_service('/motorctrl_service/readspeed')
        rospy.wait_for_service('/motorctrl_service/writepos')
        rospy.wait_for_service('/motorctrl_service/writespeed')
        rospy.loginfo(rospy.get_name() + " Found service /motorctrl_service/* .")
        self.writespeed_proxy = rospy.ServiceProxy('motorctrl_service/writespeed', motorctrl_writespeed)
        rospy.Subscriber("cmd_vel", Twist, self.callback)
        rospy.loginfo(rospy.get_name() + " subscribed to cmd_vel.")
    # # Initialize the node and name it.
    # def callback_with_accel(twist):
    #     #TODO: Something is broken. The speed jumps to top at the first stage - acceleration is not smooth. Deceleration is resonably smooth
    #     rospy.loginfo(rospy.get_name() + " Received a twist, sending command to montrol controller")
    #     readspeed_proxy = rospy.ServiceProxy('motorctrl_service/readspeed', motorctrl_readspeed)
    #     readspeed_response=readspeed_proxy()
    #     #TODO: controller to expose a single command to read pos and speed
    #     readpos_proxy = rospy.ServiceProxy('motorctrl_service/readpos', motorctrl_readpos)
    #     readpos_response=readpos_proxy()
    #
    #     speed=ENC_PER_M * twist.linear.x #Twist linear is in m/s
    #     speed_adjust=ENC_PER_RAD * twist.angular.z #Twist angular is in rad/s
    #     target_speed_left=speed-speed_adjust
    #     target_speed_right=speed+speed_adjust
    #
    #     target_speed = [target_speed_left,target_speed_left,target_speed_right,target_speed_right]
    #     writeposentries = []
    #     prev_encoder = readpos_response.encoder
    #
    #     rospy.loginfo(rospy.get_name() + " initial encoders " + str(prev_encoder))
    #
    #     # Find the speed of the wheel which differs most from the target (target) speed
    #     max_speed_diff = -1
    #     max_target_speed = -1
    #     for i in range(0, 4):
    #         speed_diff = abs(target_speed[i]-readspeed_response.speed[i])
    #         max_speed_diff = max(max_speed_diff,speed_diff)
    #         max_target_speed = max(max_target_speed,target_speed[i])
    #
    #     rospy.loginfo(rospy.get_name() + " max_speed_diff " + str(max_speed_diff))
    #     rospy.loginfo(rospy.get_name() + " max_target_speed " + str(max_target_speed))
    #
    #     secs_per_acc_step = 0 if NUM_ACCELERATION_STEPS==1 else max_speed_diff / (ACCELERATION*ENC_PER_M) / (NUM_ACCELERATION_STEPS -1)
    #     secs_per_dec_step = 0 if NUM_ACCELERATION_STEPS==1 else max_target_speed / (ACCELERATION*ENC_PER_M) / (NUM_ACCELERATION_STEPS -1)
    #     secs_target = INTERVAL_SECS - secs_per_acc_step*(NUM_ACCELERATION_STEPS -1) - secs_per_dec_step*(NUM_ACCELERATION_STEPS -1)
    #     #TODO: if secs_target is negative, shorten the other steps accordingly (or floor at 0)
    #     secs_target = max(secs_target,0)
    #
    #     rospy.loginfo(rospy.get_name() + " secs_per_acc_step "+str(secs_per_acc_step))
    #     rospy.loginfo(rospy.get_name() + " secs_per_dec_step "+str(secs_per_dec_step))
    #     rospy.loginfo(rospy.get_name() + " secs_target "+str(secs_target))
    #
    #
    #     #TODO: Allow for fast brake (maybe if received speed is 0, emergency brake)
    #     #accelerate
    #     prev_time = readpos_response.time
    #     for a in range(1,NUM_ACCELERATION_STEPS):
    #         encoder = []
    #         for i in range(0, 4):
    #             wheel_start_speed = readspeed_response.speed[i]
    #             wheel_speed = a * (target_speed[i] - wheel_start_speed) / NUM_ACCELERATION_STEPS + wheel_start_speed
    #             rospy.loginfo(rospy.get_name() + "wheel "+str(i)+" speed "+str(wheel_speed))
    #             wheel_encoder = prev_encoder[i] + wheel_speed*secs_per_acc_step
    #             encoder.append(wheel_encoder)
    #         time = add_seconds(prev_time, secs_per_acc_step)
    #         writeposentry = motorctrl_writeentry(time=time,encoder=encoder)
    #         rospy.loginfo(rospy.get_name() + " acc writeposentry " + str(writeposentry))
    #         writeposentries.append(writeposentry)
    #         prev_encoder = encoder
    #         prev_time = time
    #
    #     #remain at target speed
    #     encoder = []
    #     for i in range(0, 4):
    #         wheel_encoder = prev_encoder[i] + secs_target*target_speed[i]
    #         encoder.append(wheel_encoder)
    #     time = add_seconds(prev_time, secs_target)
    #     writeposentry = motorctrl_writeentry(time=time, encoder=encoder)
    #     writeposentries.append(writeposentry)
    #     rospy.loginfo(rospy.get_name() + " target writeposentry " + str(writeposentry))
    #     prev_encoder = encoder
    #     prev_time = time
    #
    #     #Deccelerate speed to 0
    #     for a in range(1,NUM_ACCELERATION_STEPS):
    #         encoder = []
    #         for i in range(0, 4):
    #             wheel_start_speed = target_speed[i]
    #             wheel_speed = wheel_start_speed - a * wheel_start_speed / NUM_ACCELERATION_STEPS
    #             rospy.loginfo(rospy.get_name() + "wheel "+str(i)+" speed "+str(wheel_speed))
    #             wheel_encoder = prev_encoder[i] + wheel_speed*secs_per_dec_step
    #             encoder.append(wheel_encoder)
    #         time = add_seconds(prev_time, secs_per_dec_step)
    #         writeposentry = motorctrl_writeentry(time=time,target=encoder)
    #         rospy.loginfo(rospy.get_name() + " dec writeposentry " + str(writeposentry))
    #         writeposentries.append(writeposentry)
    #         prev_encoder = encoder
    #         prev_time = time
    #
    #     #upload the data
    #     writepos_proxy = rospy.ServiceProxy('motorctrl_service/writepos', motorctrl_writepos)
    #     writepos_response=writepos_proxy(writeposentries=writeposentries)
    #
    #
    # def callback_with_pos(twist):
    #     # rospy.loginfo(rospy.get_name() + " Received a twist, sending command to montrol controller")
    #     readspeed_proxy = rospy.ServiceProxy('motorctrl_service/readspeed', motorctrl_readspeed)
    #     readspeed_response=readspeed_proxy()
    #
    #     readpos_proxy = rospy.ServiceProxy('motorctrl_service/readpos', motorctrl_readpos)
    #     readpos_response=readpos_proxy()
    #
    #     time = add_seconds(readpos_response.time, INTERVAL_SECS)
    #     speed = twist.linear.x #In m/s
    #     turn = twist.angular.z #In rad/s
    #     enc_per_interval=ENC_PER_M * speed * INTERVAL_SECS
    #     enc_adjust_per_interval=ENC_PER_RAD * turn * INTERVAL_SECS
    #     enc_per_interval_left=enc_per_interval-enc_adjust_per_interval
    #     enc_per_interval_right=enc_per_interval+enc_adjust_per_interval
    #     encoder=[
    #         readpos_response.encoder[0]+enc_per_interval_left,
    #         readpos_response.encoder[1]+enc_per_interval_left,
    #         readpos_response.encoder[2]+enc_per_interval_right,
    #         readpos_response.encoder[3]+enc_per_interval_right
    #     ]
    #     writepos_proxy = rospy.ServiceProxy('motorctrl_service/writepos', motorctrl_writepos)
    #     writeposentry=motorctrl_writeentry(time=time,target=encoder)
    #     writeposentries=[writeposentry]
    #     writepos_response=writepos_proxy(writeposentries=writeposentries)
    #
    #
    # def callback_simple(twist):
    #     now = rospy.Time.now()
    #     time = add_seconds(now, INTERVAL_SECS)
    #     speed = twist.linear.x #In m/s
    #     turn = twist.angular.z #In rad/s
    #     enc_per_sec=ENC_PER_M * speed
    #     enc_adjust_per_sec=ENC_PER_RAD * turn
    #     enc_per_sec_left=enc_per_sec-enc_adjust_per_sec
    #     enc_per_sec_right=enc_per_sec+enc_adjust_per_sec
    #     speeds=[
    #         enc_per_sec_left,
    #         enc_per_sec_left,
    #         enc_per_sec_right,
    #         enc_per_sec_right
    #     ]
    #     writespeed_proxy = rospy.ServiceProxy('motorctrl_service/writespeed', motorctrl_writespeed)
    #     writespeedentry=motorctrl_writeentry(time=time,target=speeds)
    #     writespeedentries=[writespeedentry]
    #     writespeed_response=writespeed_proxy(writespeedentries=writespeedentries)
    #
    # def callback(twist):
    #     now = rospy.Time.now()
    #     time = add_seconds(now, INTERVAL_SECS)
    #     path_speed = twist.linear.x #In m/s
    #     ang_speed = twist.angular.z #In rad/s
    #     #
    #     controller_speed, controller_delta = speed_converter.convert_to_controller(path_speed,ang_speed)
    #     # send to controller
    #     enc_per_sec=ENC_PER_M * controller_speed
    #     enc_adjust_per_sec=ENC_PER_RAD * controller_delta
    #     enc_per_sec_left=enc_per_sec-enc_adjust_per_sec
    #     enc_per_sec_right=enc_per_sec+enc_adjust_per_sec
    #     speeds=[
    #         enc_per_sec_left,
    #         enc_per_sec_left,
    #         enc_per_sec_right,
    #         enc_per_sec_right
    #     ]
    #     writespeed_proxy = rospy.ServiceProxy('motorctrl_service/writespeed', motorctrl_writespeed)
    #     writespeedentry=motorctrl_writeentry(time=time,target=speeds)
    #     writespeedentries=[writespeedentry]
    #     writespeed_response=writespeed_proxy(writespeedentries=writespeedentries)

    def add_seconds(self,time,seconds):
        add_secs=int(seconds)
        add_nsecs=(seconds-add_secs)*1000000000
        return rospy.Time(secs=time.secs+add_secs,nsecs=time.nsecs+add_nsecs)

    def callback(self,twist):
        now = rospy.Time.now()
        time = self.add_seconds(now, INTERVAL_SECS)
        path_speed = twist.linear.x #In m/s
        ang_speed = twist.angular.z #In rad/s
        #
        # Apply the limits for path speed / ang speed
        path_speed=min(path_speed,MAX_SPEED)
        path_speed=max(path_speed,-MAX_SPEED)
        ang_speed=min(ang_speed,MAX_ANG_SPEED)
        ang_speed=max(ang_speed,-MAX_ANG_SPEED)

        #
        # TODO: This is a very simple approximation to map angular speed and path speed to controller delta and speed
        # Suggestion for improvements: Either interpolate, or use linear regression to match two planes, and then intersect the two plances
        controller_speed = path_speed
        controller_delta = ang_speed / 3.657
        # send to controller
        enc_per_sec_left=enc_per_m*(controller_speed-controller_delta)
        enc_per_sec_right=enc_per_m*(controller_speed+controller_delta)
        speeds=[
            enc_per_sec_left,
            enc_per_sec_left,
            enc_per_sec_right,
            enc_per_sec_right
        ]
        writespeedentry=motorctrl_writeentry(time=time,target=speeds)
        writespeedentries=[writespeedentry]
        try:
            writespeed_response=self.writespeed_proxy(writespeedentries=writespeedentries)
        except:
            #TODO: imprve the recovery behavior
            self.writespeed_proxy = rospy.ServiceProxy('motorctrl_service/writespeed', motorctrl_writespeed)
            writespeed_response = self.writespeed_proxy(writespeedentries=writespeedentries)


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    node = Node()
    rospy.spin()


