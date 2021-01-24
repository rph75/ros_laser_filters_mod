#!/usr/bin/env python



from smbus import SMBus


MIN_BATT_VOLTAGE = 3.25 * 3
#MIN_BATT_VOLTAGE = 13

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


def init():
    global s
    s = SMBus(1)
    config = INA3221_CONFIG_ENABLE_CHAN1 | \
             INA3221_CONFIG_ENABLE_CHAN2 | \
             INA3221_CONFIG_ENABLE_CHAN3 | \
             INA3221_CONFIG_AVG1 | \
             INA3221_CONFIG_VBUS_CT2 | \
             INA3221_CONFIG_VSH_CT2 | \
             INA3221_CONFIG_MODE_2 | \
             INA3221_CONFIG_MODE_1 | \
             INA3221_CONFIG_MODE_0

    write_register_little_endian(INA3221_REG_CONFIG, config)

def write_register_little_endian(register,word):
    global s
    word = word & 0xFFFF
    # reverse configure byte from little to big endian
    lowbyte = word >> 8
    highbyte = (word & 0x00FF) << 8
    word_be = lowbyte + highbyte
    s.write_word_data(INA3221_ADDRESS,register,word_be)

def read_register_little_endian(register):
    result_be = s.read_word_data(INA3221_ADDRESS, register) & 0xFFFF
    # reverse configure byte from big to little endian
    lowbyte = (result_be & 0xFF00) >> 8
    highbyte = (result_be & 0x00FF) << 8
    result = lowbyte + highbyte
    # print "Read 16 bit Word addr =0x%x register = 0x%x switchresult = 0x%x " % (self._addr, register, switchresult)
    return result
    #TODO: Handle error

def getBusVoltage_V(channel):
    # Gets the raw bus voltage (16-bit signed integer, so +-32767)
    value = read_register_little_endian(INA3221_REG_BUSVOLTAGE_0 + channel * 2)
    if value > 32767:
        value -= 65536
    #Convert raw value to volts
    return value * 0.001

def getShuntVoltage_V(channel):
    # Gets the raw shunt voltage (16-bit signed integer, so +-32767)
    value = read_register_little_endian(INA3221_REG_SHUNTVOLTAGE_0 + channel * 2)
    if value > 32767:
        value -= 65536
    #Convert raw value to volts
    return value * 0.005 / 1000

def getCurrent_A(channel):
    valueDec = getShuntVoltage_V(channel) / SHUNT_RESISTOR_VALUES[channel]
    return valueDec;

# Main function.
if __name__ == '__main__':
    init()
    print("Voltage: "+str(getBusVoltage_V(0)))
