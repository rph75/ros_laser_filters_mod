from time import sleep
from imu import Driver
from struct import unpack
import registers
import definitions
import math
import time
from datetime import datetime

print('Trying to initialize the sensor...')
sensor = Driver(0x69)
print('Initialization done')

sensor.setMagI2Caddress(0x13<<1) #Default address of BMM150 is 0x10, but there are pull ups on SDO and CSB
sensor.mag_soft_reset()
val=sensor.readFromMagRegister(0x40)
print(str(val))
sensor.set_mag_catpure_rate(definitions.BMM_RATE_25HZ)
sensor.set_gyro_rate(definitions.GYRO_RATE_50HZ)
sensor.set_accel_rate(definitions.ACCEL_RATE_50HZ)
sensor.set_mag_rate(definitions.MAG_RATE_50HZ)
sensor.setFIFOHeaderModeEnabled(False)
sensor.setGyroFIFOEnabled(True)
sensor.setAccelFIFOEnabled(True)
sensor.setMagFIFOEnabled(True)
#val=sensor.readFromMagRegister(0x4c)
#print(str(val))

count=0
start = datetime.now()
sensor.resetFIFO()
while True:
  sleep(0.1)
  now = datetime.now()
  data=sensor.getFIFOData()
  count=count+len(data)
  print(str(now)+" Read "+str(count)+" numbers after "+str(now-start))
  print("Values are "+str(data))


while True:
  data = sensor.getMotion6()
  # fetch all gyro and acclerometer values
  print({
    'gx': data[0],
    'gy': data[1],
    'gz': data[2],
    'ax': data[3],
    'ay': data[4],
    'az': data[5]
  })
  sleep(0.1)