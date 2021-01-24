## bit field offsets and lengths
ACC_PMU_STATUS_BIT  = (4)
ACC_PMU_STATUS_LEN  = (2)
GYR_PMU_STATUS_BIT  = (2)
GYR_PMU_STATUS_LEN  = (2)
MAG_PMU_STATUS_BIT  = (0)
MAG_PMU_STATUS_LEN  = (2)
MAG_RATE_SEL_BIT   = (0)
MAG_RATE_SEL_LEN   = (4)
GYRO_RANGE_SEL_BIT  = (0)
GYRO_RANGE_SEL_LEN  = (3)
GYRO_RATE_SEL_BIT   = (0)
GYRO_RATE_SEL_LEN   = (4)
GYRO_DLPF_SEL_BIT   = (4)
GYRO_DLPF_SEL_LEN   = (2)
ACCEL_DLPF_SEL_BIT  = (4)
ACCEL_DLPF_SEL_LEN  = (3)
ACCEL_RANGE_SEL_BIT = (0)
ACCEL_RANGE_SEL_LEN = (4)
ACCEL_RATE_SEL_BIT   = (0)
ACCEL_RATE_SEL_LEN   = (4)
STATUS_FOC_RDY =       (3)
STATUS_MAG_MAN_OP =    (2)
MAG_IF_1_MANUAL_ENABLED_BIT = (7)
MAG_IF_0_MAG_RD_BURST_BIT = (0)
MAG_IF_0_MAG_RD_BURST_LEN = (2)

## Gyroscope Sensitivity Range options
# see setFullScaleGyroRange()
GYRO_RANGE_2000     = (0)    # +/- 2000 degrees/second
GYRO_RANGE_1000     = (1)    # +/- 1000 degrees/second
GYRO_RANGE_500      = (2)    # +/-  500 degrees/second
GYRO_RANGE_250      = (3)    # +/-  250 degrees/second
GYRO_RANGE_125      = (4)    # +/-  125 degrees/second

GYRO_RATE_25HZ      = (0x06)
GYRO_RATE_50HZ      = (0x07)
GYRO_RATE_100HZ      = (0x08)
GYRO_RATE_200HZ      = (0x09)
GYRO_RATE_400HZ      = (0x0A)
GYRO_RATE_800HZ      = (0x0B)
GYRO_RATE_1600HZ      = (0x0C)
GYRO_RATE_3200HZ      = (0x0D)

MAG_RATE_0_78HZ      = (0x01)
MAG_RATE_1_56HZ      = (0x02)
MAG_RATE_3_12HZ      = (0x03)
MAG_RATE_6_25HZ      = (0x04)
MAG_RATE_12_5HZ =  (0x05)
MAG_RATE_25HZ =    (0x06)
MAG_RATE_50HZ =    (0x07)
MAG_RATE_100HZ =   (0x08)
MAG_RATE_200HZ =   (0x09)
MAG_RATE_400HZ =   (0x0A)
MAG_RATE_800HZ =   (0x0B)

## Accelerometer Sensitivity Range options
# see setFullScaleAccelRange()
ACCEL_RANGE_2G      = (0X03) # +/-  2g range
ACCEL_RANGE_4G      = (0X05) # +/-  4g range
ACCEL_RANGE_8G      = (0X08) # +/-  8g range
ACCEL_RANGE_16G     = (0X0C) # +/- 16g range

ACCEL_RATE_0_78HZ      = (0x01)
ACCEL_RATE_1_56HZ      = (0x02)
ACCEL_RATE_3_12HZ      = (0x03)
ACCEL_RATE_6_25HZ      = (0x04)
ACCEL_RATE_12_5HZ      = (0x05)
ACCEL_RATE_25HZ      = (0x06)
ACCEL_RATE_50HZ      = (0x07)
ACCEL_RATE_100HZ      = (0x08)
ACCEL_RATE_200HZ      = (0x09)
ACCEL_RATE_400HZ      = (0x0A)
ACCEL_RATE_800HZ      = (0x0B)
ACCEL_RATE_1600HZ      = (0x0C)


FOC_ACC_Z_BIT       = (0)
FOC_ACC_Z_LEN       = (2)
FOC_ACC_Y_BIT       = (2)
FOC_ACC_Y_LEN       = (2)
FOC_ACC_X_BIT       = (4)
FOC_ACC_X_LEN       = (2)
FOC_GYR_EN          = (6)


FIFO_HEADER_EN_BIT = (4)
FIFO_GYR_EN_BIT = (7)
FIFO_ACC_EN_BIT = (6)
FIFO_MAG_EN_BIT = (5)

IF_MODE_MAG = (0x02)
IF_MODE_BIT = (4)
IF_MODE_LEN = (2)

#These values apply to the magnetometer
BMM_RATE_10HZ = (0x00)
BMM_RATE_2HZ =  (0x01)
BMM_RATE_6HZ =  (0x02)
BMM_RATE_8HZ =  (0x03)
BMM_RATE_15HZ = (0x04)
BMM_RATE_20HZ = (0x05)
BMM_RATE_25HZ = (0x06)
BMM_RATE_30HZ = (0x07)

BMM150_ODR_BIT = (3)