# Note:
# To minify, execute `pyminify app.py -o app.min.py --rename-globals --preserve-globals run`

# Micropython
from machine import I2C, Pin, PWM
import time
import math

# Sensors
from mpu9250 import MPU9250
from mpu6500 import MPU6500, SF_DEG_S
from vl53l0x import VL53L0X

# Display
from ssd1306 import SSD1306_I2C


I2C_CONFIG = I2C(sda=Pin(4), scl=Pin(5))
MOTION6500 = MPU6500(I2C_CONFIG, gyro_sf=SF_DEG_S)
MOTION = MPU9250(I2C_CONFIG, MOTION6500)
# LIDAR = VL53L0X(I2C_CONFIG)
DISPLAY = SSD1306_I2C(128, 64, I2C_CONFIG)
MOTOR = [
    PWM(Pin(0)),
    PWM(Pin(2)),
    PWM(Pin(13)),
    PWM(Pin(15)),
]


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
    return offset


def norm(vector):
    return math.sqrt(sum(x**2 for x in vector))


def normalize(vector):
    length = norm(vector)
    return [(x / length) for x in vector]


def dot(vector1, vector2):
    return sum((x * y) for x, y in zip(vector1, vector2))


def run_motor(motor1=None, motor2=None):
    if motor1 is not None:
        motor1 = int(motor1)
        if motor1 >= 0:
            MOTOR[0].duty(min(motor1, 1023))
            MOTOR[1].duty(0)
        else:
            MOTOR[0].duty(0)
            MOTOR[1].duty(min(-motor1, 1023))
    if motor2 is not None:
        motor2 = int(motor2)
        if motor2 >= 0:
            MOTOR[2].duty(min(motor2, 1023))
            MOTOR[3].duty(0)
        else:
            MOTOR[2].duty(0)
            MOTOR[3].duty(min(-motor2, 1023))


def run():
    # set up sensors
    gravity = calibrate(lambda: MOTION.acceleration)
    gyro_offset = calibrate(lambda: MOTION.gyro)
    gravity_unit = normalize(gravity)
    velocity = [integral() for _ in range(3)]
    position = [integral() for _ in range(3)]
    direction = integral()

    # set up motors
    for pin in MOTOR:
        pin.freq(1000)

    run_motor(0, 0)
    time.sleep(10)
    # time.sleep(10)
    # run_motor(800, -800)
    # time.sleep(3)
    # run_motor(-800, 800)
    # time.sleep(3)
    # run_motor(0, 0)
    k = 0
    ticks = time.ticks_us()
    alpha = 0
    while True:
        acceleration = with_offset(MOTION.acceleration, gravity)
        gyro = with_offset(MOTION.gyro, gyro_offset)
        current_ticks = time.ticks_us()
        step = time.ticks_diff(current_ticks, ticks) / 1_000_000

        # integrate acceleration
        for i in range(3):
            position[i](velocity[i](acceleration[i], step), step)

        # calculate and integrate angular velocity
        omega = dot(gyro, gravity_unit)
        direction(omega, step)

        if k % 10 == 0:
            alpha = .85 * alpha + (2 * direction() + 1.5 * omega)
            run_motor(800 - alpha, 800 + alpha)
            DISPLAY.fill(0)
            DISPLAY.text(str(alpha), 0, 0, 1)
            DISPLAY.show()
            # print(alpha)

        if False:
            # print(step)
            DISPLAY.fill(0)
            DISPLAY.text("Position:", 0, 0, 1)
            DISPLAY.text(
                "{:.1f} {:.1f} {:.1f}".format(*(pos() for pos in position)), 0, 8, 1
            )
            DISPLAY.text("Direction:", 0, 16, 1)
            DISPLAY.text("{:.1f}".format(direction()), 0, 24, 1)
            DISPLAY.text("Sensor data:", 0, 32, 1)
            DISPLAY.text(
                "a {:.1f} {:.1f} {:.1f}".format(*acceleration),
                0,
                40,
                1,
            )
            DISPLAY.text("r {:.1f} {:.1f} {:.1f}".format(*MOTION.gyro), 0, 48, 1)
            # DISPLAY.text("m {:.1f} {:.1f} {:.1f}".format(*MOTION.magnetic), 0, 40, 1)
            # DISPLAY.text("dist. = {:.1f}".format(LIDAR.ping()), 0, 48, 1)

            DISPLAY.show()

        k += 1
        ticks = current_ticks
        time.sleep_ms(10)
