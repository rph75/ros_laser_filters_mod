#!/usr/bin/env python

import rospy
from i2c.srv import i2c_read_word, i2c_write_word
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
# constants

MIN_BATT_VOLTAGE = 3.25 * 3

# /*=========================================================================
#    I2C ADDRESS/BITS
#    -----------------------------------------------------------------------*/
INA3221_ADDRESS = (0x40)  # 1000000 (A0+A1=GND)
INA3221_READ = (0x01)

INA3221_CONFIG_RESET = (0x8000)  # Reset Bit

INA3221_CONFIG_ENABLE_CHAN1 = (0x4000)  # Enable Channel 1
INA3221_CONFIG_ENABLE_CHAN2 = (0x2000)  # Enable Channel 2
INA3221_CONFIG_ENABLE_CHAN3 = (0x1000)  # Enable Channel 3

INA3221_CONFIG_AVG2 = (0x0800)  # AVG Samples Bit 2 - See table 3 spec
INA3221_CONFIG_AVG1 = (0x0400)  # AVG Samples Bit 1 - See table 3 spec
INA3221_CONFIG_AVG0 = (0x0200)  # AVG Samples Bit 0 - See table 3 spec

INA3221_CONFIG_VBUS_CT2 = (0x0100)  # VBUS bit 2 Conversion time - See table 4 spec
INA3221_CONFIG_VBUS_CT1 = (0x0080)  # VBUS bit 1 Conversion time - See table 4 spec
INA3221_CONFIG_VBUS_CT0 = (0x0040)  # VBUS bit 0 Conversion time - See table 4 spec

INA3221_CONFIG_VSH_CT2 = (0x0020)  # Vshunt bit 2 Conversion time - See table 5 spec
INA3221_CONFIG_VSH_CT1 = (0x0010)  # Vshunt bit 1 Conversion time - See table 5 spec
INA3221_CONFIG_VSH_CT0 = (0x0008)  # Vshunt bit 0 Conversion time - See table 5 spec

INA3221_CONFIG_MODE_2 = (0x0004)  # Operating Mode bit 2 - See table 6 spec
INA3221_CONFIG_MODE_1 = (0x0002)  # Operating Mode bit 1 - See table 6 spec
INA3221_CONFIG_MODE_0 = (0x0001)  # Operating Mode bit 0 - See table 6 spec

INA3221_REG_CONFIG = (0x00) # CONFIG REGISTER (R/W)
INA3221_REG_SHUNTVOLTAGE_0 = (0x01) #    SHUNT VOLTAGE REGISTER (R) for channel 0 (other channels are +2 / +4)
INA3221_REG_BUSVOLTAGE_0 = (0x02) #    BUS VOLTAGE REGISTER (R) fir channel 0 (other channels are +2 / +4)

# /*=========================================================================*/

#Idx = channel, value = shut resistor in ohms
SHUNT_RESISTOR_VALUES = [
    0.03333, #33.33 mOhm
    0.1, #100mOhm
    0.05 #50 mOhm
]

# The battery supply channel
BATTERY_CHANNEL=2
# The 5V channel powering the Jetson (Brain) only
BRAIN_CHANNEL=0

ORANGE_BGR = (0, 138, 230)
WHITE_BGR = (255, 255, 255)
BLACK_BGR = (0,0,0)
RED_BGR = (0,0,204)

LOG_INTERVAL = 10.0
BLINK_INTERVAL = 1.0 #Interval to blink voltage if voltage cricially low

class Node:
    # Create a callback function for the subscriber.
    def callback(self,timer):
        # Simply print out values in our custom message.
        batt_V = self.getBusVoltage_V(BATTERY_CHANNEL)
        batt_A = self.getCurrent_A(BATTERY_CHANNEL)
        total_W = batt_V * batt_A
        brain_V = self.getBusVoltage_V(BRAIN_CHANNEL)
        brain_A = self.getCurrent_A(BRAIN_CHANNEL)
        brain_W = brain_V * brain_A
        #
        # Publish battery message
        batteryState = BatteryState()
        batteryState.voltage = batt_V
        batteryState.current = batt_A
        batteryState.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        self.pub.publish(batteryState)
        #
        # Log battery status
        current_time = rospy.Time.now()
        if (current_time>self.last_log+rospy.Duration.from_sec(LOG_INTERVAL)):
            self.last_log=current_time
            if batt_V < MIN_BATT_VOLTAGE:
                rospy.logwarn(rospy.get_name() + " Battery voltage critically low {:.2f}: ".format(batt_V))
            else:
                rospy.loginfo(rospy.get_name() + " Battery voltage ok {:.2f}: ".format(batt_V))

        # Send battery image
        if (current_time>self.last_blink+rospy.Duration.from_sec(BLINK_INTERVAL)):
            self.last_blink=current_time
            self.blink_flag=not self.blink_flag

        img_color = WHITE_BGR
        text_color = BLACK_BGR
        if batt_V < MIN_BATT_VOLTAGE:
            text_color = RED_BGR
            if self.blink_flag:
                img_color = ORANGE_BGR
        img = 0xff * np.ones(shape=[200, 400, 3], dtype=np.uint8)
        img[:] = img_color
        cv2.putText(img, "Batt: {:.1f} V".format(batt_V), (25, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, text_color, 3)
        cv2.putText(img, "Brain: {:.1f} W".format(brain_W), (25, 120), cv2.FONT_HERSHEY_SIMPLEX, 1.5, BLACK_BGR, 2)
        cv2.putText(img, "Total: {:.1f} W".format(total_W), (25, 170), cv2.FONT_HERSHEY_SIMPLEX, 1.5, BLACK_BGR, 2)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img))
        return


    def write_register_little_endian(self,register,word):
        word = word & 0xFFFF
        # reverse configure byte from little to big endian
        lowbyte = word >> 8
        highbyte = (word & 0x00FF) << 8
        word_be = lowbyte + highbyte
        response=self.i2c_write_word_proxy(address=INA3221_ADDRESS,register=register,word=word_be)
        #TODO: Handle error

    def read_register_little_endian(self,register):
        response=self.i2c_read_word_proxy(address=INA3221_ADDRESS,register=register)
        result_be = response.word & 0xFFFF
        # reverse configure byte from big to little endian
        lowbyte = (result_be & 0xFF00) >> 8
        highbyte = (result_be & 0x00FF) << 8
        result = lowbyte + highbyte
        # print "Read 16 bit Word addr =0x%x register = 0x%x switchresult = 0x%x " % (self._addr, register, switchresult)
        return result
        #TODO: Handle error

    def getBusVoltage_V(self,channel):
        # Gets the raw bus voltage (16-bit signed integer, so +-32767)
        value = self.read_register_little_endian(INA3221_REG_BUSVOLTAGE_0 + channel * 2)
        if value > 32767:
            value -= 65536
        #Convert raw value to volts
        return value * 0.001

    def getShuntVoltage_V(self,channel):
        # Gets the raw shunt voltage (16-bit signed integer, so +-32767)
        value = self.read_register_little_endian(INA3221_REG_SHUNTVOLTAGE_0 + channel * 2)
        if value > 32767:
            value -= 65536
        #Convert raw value to volts
        return value * 0.005 / 1000

    def getCurrent_A(self,channel):
        valueDec = self.getShuntVoltage_V(channel) / SHUNT_RESISTOR_VALUES[channel]
        return valueDec;

    def __init__(self):
        # Initialize the node and name it.
        rospy.init_node('battery')
        #
        rospy.loginfo(rospy.get_name() + " Waiting for service /i2c_service/*...")
        rospy.wait_for_service('/i2c_service/read_word')
        rospy.wait_for_service('/i2c_service/write_word')
        rospy.loginfo(rospy.get_name() + " Found service /i2c_service/*.")
        self.i2c_write_word_proxy = rospy.ServiceProxy('/i2c_service/write_word', i2c_write_word)
        self.i2c_read_word_proxy = rospy.ServiceProxy('/i2c_service/read_word', i2c_read_word)
        #
        config = INA3221_CONFIG_ENABLE_CHAN1 | \
                 INA3221_CONFIG_ENABLE_CHAN2 | \
                 INA3221_CONFIG_ENABLE_CHAN3 | \
                 INA3221_CONFIG_AVG1 | \
                 INA3221_CONFIG_VBUS_CT2 | \
                 INA3221_CONFIG_VSH_CT2 | \
                 INA3221_CONFIG_MODE_2 | \
                 INA3221_CONFIG_MODE_1 | \
                 INA3221_CONFIG_MODE_0
        self.write_register_little_endian(INA3221_REG_CONFIG, config)
        #
        self.pub = rospy.Publisher('battery', BatteryState, queue_size=10)
        self.image_pub = rospy.Publisher("battery_image", Image, queue_size=2)
        self.last_log = rospy.Time.now()
        self.last_blink = rospy.Time.now()
        self.blink_flag = False
        self.bridge = CvBridge()
        rospy.Timer(rospy.Duration.from_sec(1.0), self.callback)
        rospy.spin()

# Main function.
if __name__ == '__main__':
    node = Node()
