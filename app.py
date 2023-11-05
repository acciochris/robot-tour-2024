# Micropython
from machine import I2C, Pin
import time

# Sensors
from mpu9250 import MPU9250
from vl53l0x import VL53L0X

# Display
from ssd1306 import SSD1306_I2C


I2C_CONFIG = I2C(sda=Pin(4), scl=Pin(5))
MOTION = MPU9250(I2C_CONFIG)
# LIDAR = VL53L0X(I2C_CONFIG)
DISPLAY = SSD1306_I2C(128, 64, I2C_CONFIG)


### Utilities
def integral(offset: float = 0.0, initial: float = 0.0, step: float | None = None):
    """Numerical integration"""

    total = offset
    last = initial
    default_step = step

    def accumulate(value: float | None = None, step: float | None = None):
        nonlocal total, last

        # don't change anything if value is not given
        if value is None:
            return total

        # fall back to default step
        if step is None:
            step = default_step

        # error if neither step nor default_step is specified
        if step is None:
            raise ValueError("No step specified")

        total += step * (last + value) / 2
        last = value

        return total

    return accumulate


def with_offset(values, offset):
    return tuple((values[i] - offset[i]) for i in range(3))


def calibrate(func):
    offset = [0.0, 0.0, 0.0]
    for _ in range(100):
        x, y, z = func()
        offset[0] += x
        offset[1] += y
        offset[2] += z
        time.sleep_ms(10)
    offset[0] /= 100
    offset[1] /= 100
    offset[2] /= 100
    return tuple(offset)


def run():
    gravity = calibrate(lambda: MOTION.acceleration)
    velocity = [integral() for _ in range(3)]
    position = [integral() for _ in range(3)]

    k = 0
    ticks = time.ticks_us()
    while True:
        acceleration = with_offset(MOTION.acceleration, gravity)
        current_ticks = time.ticks_us()
        step = time.ticks_diff(current_ticks, ticks) / 1_000_000
        for i in range(3):
            position[i](velocity[i](acceleration[i], step), step)

        if k == 20:
            k = 0
            print(step)
            DISPLAY.fill(0)
            DISPLAY.text("Position:", 0, 0, 1)
            DISPLAY.text(
                "{:.1f} {:.1f} {:.1f}".format(*(pos() for pos in position)), 0, 8, 1
            )
            DISPLAY.text("Sensor data:", 0, 16, 1)
            DISPLAY.text(
                "a {:.1f} {:.1f} {:.1f}".format(*acceleration),
                0,
                24,
                1,
            )
            DISPLAY.text("r {:.1f} {:.1f} {:.1f}".format(*MOTION.gyro), 0, 32, 1)
            # DISPLAY.text("m {:.1f} {:.1f} {:.1f}".format(*MOTION.magnetic), 0, 40, 1)
            # DISPLAY.text("dist. = {:.1f}".format(LIDAR.ping()), 0, 48, 1)

            DISPLAY.show()

        k += 1
        ticks = current_ticks
        time.sleep_ms(10)
