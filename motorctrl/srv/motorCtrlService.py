#!/usr/bin/env python
import rospy
from i2c.srv import i2c_frame, i2c_frameResponse
from motorctrl.srv import motorctrl_readspeed, motorctrl_readspeedResponse, motorctrl_readpos, motorctrl_readposResponse, motorctrl_writepos, motorctrl_writeposResponse, motorctrl_writespeed, motorctrl_writespeedResponse
from motorctrl.srv import motorctrl_grippercommand, motorctrl_grippercommandResponse

servoPwmFreq = 200
full_duty=10000         #Number to be sent as full duty cycle
servo_pulse_lo=500         #Pulse length for lowest position, in us
servo_pulse_hi=2400       #Pulse length for highest position, in us

MOTORCTRL_ADDRESS = (0x33)

enc_per_m=11500 #encoder units per m #TODO: centralize this value (shared with odometry)

#TODO: Move this all to parameters
motorPwmFreq = 17500
motorTimeout = -1
timePID = 15  # Run PID once every x ms
kp = 600
ki = 20
kd = 0
quotient = 100  # P,I,D are scaled by factor 100

#TODO: Currently we set all params for position to 0 - this means position will be controlled using speed,
#and final pos adjustment is disabled
timePID_pos = 200
kp_pos = 0
ki_pos = 0
kd_pos = 0
quotient_pos = 10

motor_now_ms = -1  # Difference of motor time to current time (current time - motor time)

default_pos = 0.25 # Servo position when gripper is up (default position)

accel_interval = 50 #How often to check the acceleration (in ms) - too low value may result in oscillation
max_accel = 12000 # max acceleration (in enc per sec^2)

def readspeed(request):
    # TODO: Exception handling
    global motor_now_ms
    payload = [0x34]   # Read speed time
    i2c_frame_proxy = rospy.ServiceProxy('/i2c_service/frame', i2c_frame)
    i2c_response=i2c_frame_proxy(address=MOTORCTRL_ADDRESS,req_data=payload)
    reply_data=i2c_response.resp_data
    #
    verify_reply_length(reply_data, 20)
    idx = 0
    response = motorctrl_readspeedResponse()
    response.speed=[0,0,0,0]
    motor_time, idx = read_int(reply_data, idx)
    time_ms = motor_now_ms + motor_time
    response.time = rospy.Time((int)(time_ms/1000),(time_ms%1000)*1000000)
    response.speed[0], idx = read_int(reply_data, idx)
    response.speed[1], idx = read_int(reply_data, idx)
    response.speed[2], idx = read_int(reply_data, idx)
    response.speed[3], idx = read_int(reply_data, idx)
    response.speed[2] = 0 - response.speed[2] #Flip sign for these motors
    response.speed[3] = 0 - response.speed[3] #Flip sign for these motors
    return response

def readpos(request):
    # TODO: Exception handling
    global motor_now_ms
    payload = [0x30]   # Read pos and time
    i2c_frame_proxy = rospy.ServiceProxy('/i2c_service/frame', i2c_frame)
    i2c_response=i2c_frame_proxy(address=MOTORCTRL_ADDRESS,req_data=payload)
    reply_data=i2c_response.resp_data
    #
    verify_reply_length(reply_data, 20)
    idx = 0
    response = motorctrl_readposResponse()
    response.encoder=[0,0,0,0]
    motor_time, idx = read_int(reply_data, idx)
    time_ms = motor_now_ms + motor_time
    response.time = rospy.Time((int)(time_ms/1000),(time_ms%1000)*1000000)
    response.encoder[0], idx = read_int(reply_data, idx)
    response.encoder[1], idx = read_int(reply_data, idx)
    response.encoder[2], idx = read_int(reply_data, idx)
    response.encoder[3], idx = read_int(reply_data, idx)
    response.encoder[2] = 0 - response.encoder[2] #Flip sign for these motors
    response.encoder[3] = 0 - response.encoder[3] #Flip sign for these motors
    return response

def writepos(request):
    global motor_now_ms
    rospy.loginfo(rospy.get_name() + " write pos command received")
    payload = [0x00]  # Send target position
    append_byte(payload, len(request.writeposentries))  # Send n targets
    for i in range(0, len(request.writeposentries)):
        entry = request.writeposentries[i]
        time_ms = entry.time.secs*1000 + entry.time.nsecs/1000000
        motor_time = time_ms - motor_now_ms
        append_int(payload, motor_time)
        append_int(payload, entry.target[0])
        append_int(payload, entry.target[1])
        append_int(payload, 0 - entry.target[2])  #Flip sign for these motors
        append_int(payload, 0 - entry.target[3])  #Flip sign for these motors
    i2c_frame_proxy = rospy.ServiceProxy('/i2c_service/frame', i2c_frame)
    i2c_response=i2c_frame_proxy(address=MOTORCTRL_ADDRESS,req_data=payload)
    reply_data=i2c_response.resp_data
    verify_reply_length(reply_data, 0)
    response = motorctrl_writeposResponse()
    return response

def writespeed(request):
    #TODO: recycle code with writepos
    rospy.loginfo(rospy.get_name() + " write speed command received")
    global motor_now_ms
    payload = [0x01]  # Send target speed
    append_byte(payload, len(request.writespeedentries))  # Send n targets
    for i in range(0, len(request.writespeedentries)):
        entry = request.writespeedentries[i]
        time_ms = entry.time.secs*1000 + entry.time.nsecs/1000000
        motor_time = time_ms - motor_now_ms
        append_int(payload, motor_time)
        append_int(payload, entry.target[0])
        append_int(payload, entry.target[1])
        append_int(payload, 0 - entry.target[2])  #Flip sign for these motors
        append_int(payload, 0 - entry.target[3])  #Flip sign for these motors
    i2c_frame_proxy = rospy.ServiceProxy('/i2c_service/frame', i2c_frame)
    i2c_response=i2c_frame_proxy(address=MOTORCTRL_ADDRESS,req_data=payload)
    reply_data=i2c_response.resp_data
    verify_reply_length(reply_data, 0)
    response = motorctrl_writespeedResponse()
    return response

def grippercommand(request):
    if request.action == 'MOVE':
        gripper_to_position(request.position, request.duration)
    elif request.action =='OPEN':
        open_gripper(request.duration)
    elif request.action == 'CLOSE':
        close_gripper(request.duration)
    response = motorctrl_grippercommandResponse()
    return response

def verify_reply_length(reply, expected_len):
    if len(reply) !=expected_len :
        raise Exception("Expected payload of size "+expected_len+", found "+len(reply))

def read_int(b,idx):
    v = ord(b[idx]);
    v = (v<<8) | ord(b[idx+1]);
    v = (v<<8) | ord(b[idx+2]);
    v = (v<<8) | ord(b[idx+3]);
    if v>=0x80000000:
        v-=0x100000000;
    return v, idx+4;

def append_int(b,v):
    b.append(v >> 24 & 0xFF)
    b.append(v >> 16 & 0xFF)
    b.append(v >> 8 & 0xFF)
    b.append(v & 0xFF)

def append_byte(b,v):
    b.append(v)

def get_time_offset():
    global motor_now_ms
    motor_now_ms = 0
    i2c_frame_proxy = rospy.ServiceProxy('/i2c_service/frame', i2c_frame)
    payload = [0x30]  # Read pos and time
    for i in range(0,10):
        # Loop a few times, and take the smallest difference
        now = rospy.Time.now()
        i2c_response = i2c_frame_proxy(address=MOTORCTRL_ADDRESS, req_data=payload)
        motor_time, idx = read_int(i2c_response.resp_data, 0)
        now_ms = now.secs*1000 + now.nsecs/1000000
        diff_ms = now_ms - motor_time
        if motor_now_ms==0 or diff_ms < motor_now_ms:
            motor_now_ms = diff_ms
            rospy.loginfo(rospy.get_name() + " computer time is   "+str(now_ms))
            rospy.loginfo(rospy.get_name() + " controller time is "+str(motor_time))
            rospy.loginfo(rospy.get_name() + " offset is " + str(motor_now_ms))


def upload_pid(timePID,quotient,p,i,d,timePID_pos,quotient_pos,p_pos,i_pos,d_pos):
    rospy.loginfo(rospy.get_name() + " Uploading P = %d, I = %d, D = %d, P_pos = %d, I_pos = %d, D_pos = %d" % (p, i, d, p_pos, i_pos, d_pos))
    payload = [0xf0]
    append_int(payload,timePID)
    append_int(payload,quotient)
    append_int(payload,p)
    append_int(payload,i)
    append_int(payload,d)
    append_int(payload,timePID_pos)
    append_int(payload,quotient_pos)
    append_int(payload,p_pos)
    append_int(payload,i_pos)
    append_int(payload,d_pos)

    i2c_frame_proxy = rospy.ServiceProxy('/i2c_service/frame', i2c_frame)
    i2c_response=i2c_frame_proxy(address=MOTORCTRL_ADDRESS,req_data=payload)
    reply_data=i2c_response.resp_data
    verify_reply_length(reply_data,0)
    rospy.loginfo(rospy.get_name() + " Uploading of PID values completed.")

def initController():
    rospy.loginfo(rospy.get_name() + " Initializing controller")
    set_motor_pwm(motorPwmFreq)
    set_motor_timeout(motorTimeout)
    set_max_accel(accel_interval,max_accel)
    upload_pid(timePID,quotient,kp,ki,kd,timePID_pos,quotient_pos,kp_pos,ki_pos,kd_pos)
    #TODO: Periodically update the time offset, as there may be a drift in micro controller time
    get_time_offset()
    rospy.loginfo(rospy.get_name() + " Controller initialized.")

def set_motor_pwm(freq):
    payload = [0xf4]  # Write PWM frequency
    append_int(payload, freq)
    i2c_frame_proxy = rospy.ServiceProxy('/i2c_service/frame', i2c_frame)
    i2c_response=i2c_frame_proxy(address=MOTORCTRL_ADDRESS,req_data=payload)
    reply_data=i2c_response.resp_data
    verify_reply_length(reply_data,0)

def set_max_accel(interval,accel):
    payload = [0xfc]  # Write max accel
    append_int(payload, interval)
    append_int(payload, accel)
    i2c_frame_proxy = rospy.ServiceProxy('/i2c_service/frame', i2c_frame)
    i2c_response=i2c_frame_proxy(address=MOTORCTRL_ADDRESS,req_data=payload)
    reply_data=i2c_response.resp_data
    verify_reply_length(reply_data,0)


def set_motor_timeout(timeout):
    payload = [0xf8]  # Write motor timeout
    append_int(payload, timeout)
    i2c_frame_proxy = rospy.ServiceProxy('/i2c_service/frame', i2c_frame)
    i2c_response=i2c_frame_proxy(address=MOTORCTRL_ADDRESS,req_data=payload)
    reply_data=i2c_response.resp_data
    verify_reply_length(reply_data,0)

def initGripper(default_pos):
    rospy.loginfo(rospy.get_name() + "Initializing gripper servo with PWM frequency {}".format(servoPwmFreq))
    gripper_to_position(default_pos, rospy.Duration.from_sec(0)) #Immediately go to defined start position
    payload = [0x20]  # Write Servo PWM frequency
    append_int(payload,servoPwmFreq)
    open_gripper(rospy.Duration.from_sec(1.5)) #Gripper open by default
    i2c_frame_proxy = rospy.ServiceProxy('/i2c_service/frame', i2c_frame)
    i2c_response=i2c_frame_proxy(address=MOTORCTRL_ADDRESS,req_data=payload)
    reply_data=i2c_response.resp_data
    verify_reply_length(reply_data,0)
    rospy.loginfo(rospy.get_name() + " Gripper initialized.")

#posotion: value betweeo 0 and 1, where 0 corresponds to servoPosLow and 1 to servoPosHigh
#time: Time to get to the position, in ms
def gripper_to_position( position, duration):
    pulse_length= servo_pulse_lo + position * (servo_pulse_hi - servo_pulse_lo)
    duty=(int)(pulse_length*servoPwmFreq * full_duty / 1000000)
    payload = [0x10]  # Go to servo position
    append_int(payload,duty)
    append_int(payload,(int)(duration.to_sec()*1000)) #in milliseconds
    i2c_frame_proxy = rospy.ServiceProxy('/i2c_service/frame', i2c_frame)
    i2c_response=i2c_frame_proxy(address=MOTORCTRL_ADDRESS,req_data=payload)
    reply_data=i2c_response.resp_data
    verify_reply_length(reply_data,0)

def open_gripper(active_time):
    payload = [0x14]  # Open gripper
    append_int(payload,(int)(active_time.to_sec()*1000)) #in milliseconds
    i2c_frame_proxy = rospy.ServiceProxy('/i2c_service/frame', i2c_frame)
    i2c_response=i2c_frame_proxy(address=MOTORCTRL_ADDRESS,req_data=payload)
    reply_data=i2c_response.resp_data
    verify_reply_length(reply_data,0)

def close_gripper(active_time):
    payload = [0x15]  # Close gripper
    append_int(payload,(int)(active_time.to_sec()*1000)) #in milliseconds
    i2c_frame_proxy = rospy.ServiceProxy('/i2c_service/frame', i2c_frame)
    i2c_response=i2c_frame_proxy(address=MOTORCTRL_ADDRESS,req_data=payload)
    reply_data=i2c_response.resp_data
    verify_reply_length(reply_data,0)

# Main function.
if __name__ == '__main__':
    rospy.init_node('motorctrl_service')
    rospy.loginfo(rospy.get_name() + " Waiting for service /i2c_service/frame...")
    rospy.wait_for_service('/i2c_service/frame')
    rospy.loginfo(rospy.get_name() + " Found service /i2c_service/frame.")
    initController()
    initGripper(default_pos)
    service=rospy.Service('motorctrl_service/readspeed',motorctrl_readspeed,readspeed)
    service=rospy.Service('motorctrl_service/readpos',motorctrl_readpos,readpos)
    service=rospy.Service('motorctrl_service/writepos',motorctrl_writepos,writepos)
    service=rospy.Service('motorctrl_service/writespeed',motorctrl_writespeed,writespeed)
    service=rospy.Service('motorctrl_service/grippercommand',motorctrl_grippercommand,grippercommand)
    rospy.spin()

