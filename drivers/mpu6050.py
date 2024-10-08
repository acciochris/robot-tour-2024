import time
from ustruct import unpack
from micropython import const

# constants extracted from
# https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.h

MPU6050_ACCEL_FS_16 = const(0x03)
MPU6050_ACCEL_FS_2 = const(0x00)
MPU6050_ACCEL_FS_4 = const(0x01)
MPU6050_ACCEL_FS_8 = const(0x02)
MPU6050_ACONFIG_ACCEL_HPF_BIT = const(2)
MPU6050_ACONFIG_ACCEL_HPF_LENGTH = const(3)
MPU6050_ACONFIG_AFS_SEL_BIT = const(4)
MPU6050_ACONFIG_AFS_SEL_LENGTH = const(2)
MPU6050_ACONFIG_XA_ST_BIT = const(7)
MPU6050_ACONFIG_YA_ST_BIT = const(6)
MPU6050_ACONFIG_ZA_ST_BIT = const(5)
MPU6050_ADDRESS_AD0_HIGH = const(0x69)
MPU6050_ADDRESS_AD0_LOW = const(0x68)
MPU6050_CFG_DLPF_CFG_BIT = const(2)
MPU6050_CFG_DLPF_CFG_LENGTH = const(3)
MPU6050_CFG_EXT_SYNC_SET_BIT = const(5)
MPU6050_CFG_EXT_SYNC_SET_LENGTH = const(3)
MPU6050_CLOCK_INTERNAL = const(0x00)
MPU6050_CLOCK_KEEP_RESET = const(0x07)
MPU6050_CLOCK_PLL_EXT19M = const(0x05)
MPU6050_CLOCK_PLL_EXT32K = const(0x04)
MPU6050_CLOCK_PLL_XGYRO = const(0x01)
MPU6050_CLOCK_PLL_YGYRO = const(0x02)
MPU6050_CLOCK_PLL_ZGYRO = const(0x03)
MPU6050_DEFAULT_ADDRESS = const(MPU6050_ADDRESS_AD0_LOW)
MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT = const(7)
MPU6050_DETECT_ACCEL_ON_DELAY_BIT = const(5)
MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH = const(2)
MPU6050_DETECT_DECREMENT_1 = const(0x1)
MPU6050_DETECT_DECREMENT_2 = const(0x2)
MPU6050_DETECT_DECREMENT_4 = const(0x3)
MPU6050_DETECT_DECREMENT_RESET = const(0x0)
MPU6050_DETECT_FF_COUNT_BIT = const(3)
MPU6050_DETECT_FF_COUNT_LENGTH = const(2)
MPU6050_DETECT_MOT_COUNT_BIT = const(1)
MPU6050_DETECT_MOT_COUNT_LENGTH = const(2)
MPU6050_DHPF_0P63 = const(0x04)
MPU6050_DHPF_1P25 = const(0x03)
MPU6050_DHPF_2P5 = const(0x02)
MPU6050_DHPF_5 = const(0x01)
MPU6050_DHPF_HOLD = const(0x07)
MPU6050_DHPF_RESET = const(0x00)
MPU6050_DLPF_BW_10 = const(0x05)
MPU6050_DLPF_BW_188 = const(0x01)
MPU6050_DLPF_BW_20 = const(0x04)
MPU6050_DLPF_BW_256 = const(0x00)
MPU6050_DLPF_BW_42 = const(0x03)
MPU6050_DLPF_BW_5 = const(0x06)
MPU6050_DLPF_BW_98 = const(0x02)
MPU6050_DMPINT_0_BIT = const(0)
MPU6050_DMPINT_1_BIT = const(1)
MPU6050_DMPINT_2_BIT = const(2)
MPU6050_DMPINT_3_BIT = const(3)
MPU6050_DMPINT_4_BIT = const(4)
MPU6050_DMPINT_5_BIT = const(5)
MPU6050_GCONFIG_FS_SEL_BIT = const(4)
MPU6050_GCONFIG_FS_SEL_LENGTH = const(2)
MPU6050_GYRO_FS_1000 = const(0x02)
MPU6050_GYRO_FS_2000 = const(0x03)
MPU6050_GYRO_FS_250 = const(0x00)
MPU6050_GYRO_FS_500 = const(0x01)
MPU6050_INTCFG_CLKOUT_EN_BIT = const(0)
MPU6050_INTCFG_FSYNC_INT_EN_BIT = const(2)
MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT = const(3)
MPU6050_INTCFG_INT_LEVEL_BIT = const(7)
MPU6050_INTCFG_INT_OPEN_BIT = const(6)
MPU6050_INTCFG_INT_RD_CLEAR_BIT = const(4)
MPU6050_INTCFG_LATCH_INT_EN_BIT = const(5)
MPU6050_INTCLEAR_ANYREAD = const(0x01)
MPU6050_INTCLEAR_STATUSREAD = const(0x00)
MPU6050_INTDRV_OPENDRAIN = const(0x01)
MPU6050_INTDRV_PUSHPULL = const(0x00)
MPU6050_INTERRUPT_DATA_RDY_BIT = const(0)
MPU6050_INTERRUPT_DMP_INT_BIT = const(1)
MPU6050_INTERRUPT_FF_BIT = const(7)
MPU6050_INTERRUPT_FIFO_OFLOW_BIT = const(4)
MPU6050_INTERRUPT_MOT_BIT = const(6)
MPU6050_INTERRUPT_PLL_RDY_INT_BIT = const(2)
MPU6050_INTERRUPT_ZMOT_BIT = const(5)
MPU6050_INTLATCH_50USPULSE = const(0x00)
MPU6050_INTLATCH_WAITCLEAR = const(0x01)
MPU6050_INTMODE_ACTIVEHIGH = const(0x00)
MPU6050_INTMODE_ACTIVELOW = const(0x01)
MPU6050_MOTION_MOT_XNEG_BIT = const(7)
MPU6050_MOTION_MOT_XPOS_BIT = const(6)
MPU6050_MOTION_MOT_YNEG_BIT = const(5)
MPU6050_MOTION_MOT_YPOS_BIT = const(4)
MPU6050_MOTION_MOT_ZNEG_BIT = const(3)
MPU6050_MOTION_MOT_ZPOS_BIT = const(2)
MPU6050_MOTION_MOT_ZRMOT_BIT = const(0)
MPU6050_PATHRESET_ACCEL_RESET_BIT = const(1)
MPU6050_PATHRESET_GYRO_RESET_BIT = const(2)
MPU6050_PATHRESET_TEMP_RESET_BIT = const(0)
MPU6050_PWR1_CLKSEL_BIT = const(2)
MPU6050_PWR1_CLKSEL_LENGTH = const(3)
MPU6050_PWR1_CYCLE_BIT = const(5)
MPU6050_PWR1_DEVICE_RESET_BIT = const(7)
MPU6050_PWR1_SLEEP_BIT = const(6)
MPU6050_PWR1_TEMP_DIS_BIT = const(3)
MPU6050_PWR2_LP_WAKE_CTRL_BIT = const(7)
MPU6050_PWR2_LP_WAKE_CTRL_LENGTH = const(2)
MPU6050_PWR2_STBY_XA_BIT = const(5)
MPU6050_PWR2_STBY_XG_BIT = const(2)
MPU6050_PWR2_STBY_YA_BIT = const(4)
MPU6050_PWR2_STBY_YG_BIT = const(1)
MPU6050_PWR2_STBY_ZA_BIT = const(3)
MPU6050_PWR2_STBY_ZG_BIT = const(0)
MPU6050_RA_ACCEL_CONFIG = const(0x1C)
MPU6050_RA_ACCEL_XOUT_H = const(0x3B)
MPU6050_RA_ACCEL_XOUT_L = const(0x3C)
MPU6050_RA_ACCEL_YOUT_H = const(0x3D)
MPU6050_RA_ACCEL_YOUT_L = const(0x3E)
MPU6050_RA_ACCEL_ZOUT_H = const(0x3F)
MPU6050_RA_ACCEL_ZOUT_L = const(0x40)
MPU6050_RA_BANK_SEL = const(0x6D)
MPU6050_RA_CONFIG = const(0x1A)
MPU6050_RA_DMP_CFG_1 = const(0x70)
MPU6050_RA_DMP_CFG_2 = const(0x71)
MPU6050_RA_DMP_INT_STATUS = const(0x39)
MPU6050_RA_FF_DUR = const(0x1E)
MPU6050_RA_FF_THR = const(0x1D)
MPU6050_RA_FIFO_COUNTH = const(0x72)
MPU6050_RA_FIFO_COUNTL = const(0x73)
MPU6050_RA_FIFO_EN = const(0x23)
MPU6050_RA_FIFO_R_W = const(0x74)
MPU6050_RA_GYRO_CONFIG = const(0x1B)
MPU6050_RA_GYRO_XOUT_H = const(0x43)
MPU6050_RA_GYRO_XOUT_L = const(0x44)
MPU6050_RA_GYRO_YOUT_H = const(0x45)
MPU6050_RA_GYRO_YOUT_L = const(0x46)
MPU6050_RA_GYRO_ZOUT_H = const(0x47)
MPU6050_RA_GYRO_ZOUT_L = const(0x48)
MPU6050_RA_INT_ENABLE = const(0x38)
MPU6050_RA_INT_PIN_CFG = const(0x37)
MPU6050_RA_INT_STATUS = const(0x3A)
MPU6050_RA_MEM_R_W = const(0x6F)
MPU6050_RA_MEM_START_ADDR = const(0x6E)
MPU6050_RA_MOT_DETECT_CTRL = const(0x69)
MPU6050_RA_MOT_DETECT_STATUS = const(0x61)
MPU6050_RA_MOT_DUR = const(0x20)
MPU6050_RA_MOT_THR = const(0x1F)
MPU6050_RA_PWR_MGMT_1 = const(0x6B)
MPU6050_RA_PWR_MGMT_2 = const(0x6C)
MPU6050_RA_SELF_TEST_A = const(0x10)
MPU6050_RA_SELF_TEST_X = const(0x0D)
MPU6050_RA_SELF_TEST_Y = const(0x0E)
MPU6050_RA_SELF_TEST_Z = const(0x0F)
MPU6050_RA_SIGNAL_PATH_RESET = const(0x68)
MPU6050_RA_SMPLRT_DIV = const(0x19)
MPU6050_RA_TEMP_OUT_H = const(0x41)
MPU6050_RA_TEMP_OUT_L = const(0x42)
MPU6050_RA_USER_CTRL = const(0x6A)
MPU6050_RA_WHO_AM_I = const(0x75)
MPU6050_RA_XA_OFFS_H = const(0x06)
MPU6050_RA_XA_OFFS_L_TC = const(0x07)
MPU6050_RA_X_FINE_GAIN = const(0x03)
MPU6050_RA_XG_OFFS_TC = const(0x00)
MPU6050_RA_XG_OFFS_USRH = const(0x13)
MPU6050_RA_XG_OFFS_USRL = const(0x14)
MPU6050_RA_YA_OFFS_H = const(0x08)
MPU6050_RA_YA_OFFS_L_TC = const(0x09)
MPU6050_RA_Y_FINE_GAIN = const(0x04)
MPU6050_RA_YG_OFFS_TC = const(0x01)
MPU6050_RA_YG_OFFS_USRH = const(0x15)
MPU6050_RA_YG_OFFS_USRL = const(0x16)
MPU6050_RA_ZA_OFFS_H = const(0x0A)
MPU6050_RA_ZA_OFFS_L_TC = const(0x0B)
MPU6050_RA_Z_FINE_GAIN = const(0x05)
MPU6050_RA_ZG_OFFS_TC = const(0x02)
MPU6050_RA_ZG_OFFS_USRH = const(0x17)
MPU6050_RA_ZG_OFFS_USRL = const(0x18)
MPU6050_RA_ZRMOT_DUR = const(0x22)
MPU6050_RA_ZRMOT_THR = const(0x21)
MPU6050_TC_OFFSET_BIT = const(6)
MPU6050_TC_OFFSET_LENGTH = const(6)
MPU6050_TC_OTP_BNK_VLD_BIT = const(0)
MPU6050_TC_PWR_MODE_BIT = const(7)
MPU6050_TEMP_FIFO_EN_BIT = const(7)
MPU6050_USERCTRL_DMP_EN_BIT = const(7)
MPU6050_USERCTRL_DMP_RESET_BIT = const(3)
MPU6050_USERCTRL_FIFO_EN_BIT = const(6)
MPU6050_USERCTRL_FIFO_RESET_BIT = const(2)
MPU6050_USERCTRL_SIG_COND_RESET_BIT = const(0)
MPU6050_VDDIO_LEVEL_VDD = const(1)
MPU6050_VDDIO_LEVEL_VLOGIC = const(0)
MPU6050_WAKE_FREQ_10 = const(0x3)
MPU6050_WAKE_FREQ_1P25 = const(0x0)
MPU6050_WAKE_FREQ_2P5 = const(0x1)
MPU6050_WAKE_FREQ_5 = const(0x2)
MPU6050_WHO_AM_I_BIT = const(6)
MPU6050_WHO_AM_I_LENGTH = const(6)
MPU6050_XG_FIFO_EN_BIT = const(6)
MPU6050_YG_FIFO_EN_BIT = const(5)
MPU6050_ZG_FIFO_EN_BIT = const(4)
MPU_ADDR = const(MPU6050_DEFAULT_ADDRESS)
MPU_DATA_RDY_PIN = const(14)
MPU_SCL_PIN = const(13)
MPU_SDA_PIN = const(12)


DEFAULT_SAMPLE_RATE = 0x20

ACCEL_RANGE = [2, 4, 8, 16]
GYRO_RANGE = [250, 500, 1000, 2000]

class MPU6050:
    stable_reading_timeout = 10
    max_gyro_variance = 5

    def __init__(self, i2c, rate=None, address=None):
        self.rate = rate or DEFAULT_SAMPLE_RATE
        self.address = address or MPU6050_DEFAULT_ADDRESS
        self.i2c = i2c

        self.buffer = bytearray(16)
        self.bytebuf = memoryview(self.buffer[0:1])
        self.wordbuf = memoryview(self.buffer[0:2])
        self.sensors = bytearray(6)

        self.init_device()

    def write_byte(self, reg, val):
        self.bytebuf[0] = val
        self.i2c.writeto_mem(self.address, reg, self.bytebuf)

    def read_byte(self, reg):
        self.i2c.readfrom_mem_into(self.address, reg, self.bytebuf)
        return self.bytebuf[0]

    def set_bitfield(self, reg, pos, length, val):
        old = self.read_byte(reg)
        shift = pos - length + 1
        mask = (2**length - 1) << shift
        new = (old & ~mask) | (val << shift)
        self.write_byte(reg, new)

    def identify(self):
        val = self.read_byte(MPU6050_RA_WHO_AM_I)
        if val != MPU6050_ADDRESS_AD0_LOW:
            raise OSError(f"No mpu6050 at address {self.address}")

    def reset(self):
        self.write_byte(MPU6050_RA_PWR_MGMT_1, (1 << MPU6050_PWR1_DEVICE_RESET_BIT))
        time.sleep_ms(100)  # type: ignore[attr-defined]

        self.write_byte(
            MPU6050_RA_SIGNAL_PATH_RESET,
            (
                (1 << MPU6050_PATHRESET_GYRO_RESET_BIT)
                | (1 << MPU6050_PATHRESET_ACCEL_RESET_BIT)
                | (1 << MPU6050_PATHRESET_TEMP_RESET_BIT)
            ),
        )
        time.sleep_ms(100)  # type: ignore[attr-defined]

    def init_device(self):
        self.identify()

        # disable sleep mode and select clock source
        self.write_byte(MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO)

        # enable all sensors
        self.write_byte(MPU6050_RA_PWR_MGMT_2, 0)

        # set sampling rate
        self.write_byte(MPU6050_RA_SMPLRT_DIV, self.rate)

        # enable dlpf
        self.write_byte(MPU6050_RA_CONFIG, 1)

        # explicitly set accel/gyro range
        self.set_accel_range(MPU6050_ACCEL_FS_2)
        self.set_gyro_range(MPU6050_GYRO_FS_250)

    def set_gyro_range(self, fsr):
        self.gyro_range = GYRO_RANGE[fsr]
        self.set_bitfield(
            MPU6050_RA_GYRO_CONFIG,
            MPU6050_GCONFIG_FS_SEL_BIT,
            MPU6050_GCONFIG_FS_SEL_LENGTH,
            fsr,
        )

    def set_accel_range(self, fsr):
        self.accel_range = ACCEL_RANGE[fsr]
        self.set_bitfield(
            MPU6050_RA_ACCEL_CONFIG,
            MPU6050_ACONFIG_AFS_SEL_BIT,
            MPU6050_ACONFIG_AFS_SEL_LENGTH,
            fsr,
        )

    @property
    def acceleration(self):
        self.i2c.readfrom_mem_into(
            self.address, MPU6050_RA_ACCEL_XOUT_H, self.sensors
        )

        data = unpack(">hhh", self.sensors)
        return tuple(x / (65536 // self.accel_range // 2) * 9.80665 for x in data)

    @property
    def gyro(self):
        self.i2c.readfrom_mem_into(self.address, MPU6050_RA_GYRO_XOUT_H, self.sensors)

        data = unpack(">hhh", self.sensors)
        return tuple(x / (65536 // self.gyro_range // 2) for x in data)

    def set_dhpf_mode(self, bandwidth):
        self.set_bitfield(
            MPU6050_RA_ACCEL_CONFIG,
            MPU6050_ACONFIG_ACCEL_HPF_BIT,
            MPU6050_ACONFIG_ACCEL_HPF_LENGTH,
            bandwidth,
        )

    # 0 Reset
    # 1 On @ 5 Hz
    # 2 On @ 2.5 Hz
    # 3 On @ 1.25 Hz
    # 4 On @ 0.63 Hz
    # 7 Hold
    def get_dhpf_mode(self):
        return self.read_byte(MPU6050_RA_ACCEL_CONFIG)

    def set_motion_detection_threshold(self, threshold):
        self.write_byte(MPU6050_RA_MOT_THR, threshold)

    def set_motion_detection_duration(self, duration):
        self.write_byte(MPU6050_RA_MOT_DUR, duration)

    def enable_motion_interrupt(self):
        self.set_bitfield(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_MOT_BIT, 1, 1)

    def disable_motion_interrupt(self):
        self.set_bitfield(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_MOT_BIT, 1, 0)
