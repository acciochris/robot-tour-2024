# Note:
# To minify, execute `pyminify app.py -o app.min.py --remove-literal-statements --rename-globals --preserve-globals run`

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

ACTIONS = "FFLR"

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

    def accumulate(
        value: float | None = None, step: float | None = None, reset: bool = False
    ):
        nonlocal total, last

        # reset if requested
        if reset:
            total = offset
            last = initial
            return total

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
    return [(values[i] - offset[i]) for i in range(3)]


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


def eval_vec(vec):
    return [ele() for ele in vec]


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


def _actions_to_ir(actions):
    """Returns a generator that yields the next action"""

    i = 0
    done = False
    while True:
        if done:
            break

        action = actions[i]

        j = 0
        while actions[i + j] == action:
            j += 1
            if (i + j) >= len(actions):
                done = True
                break

        if action == "F":
            yield {"op": "reset"}
            yield {"op": "transition", "start": 0, "stop": 800, "step": 20}
            yield {"op": "hold-forward", "value": 800, "distance": j / 2}
            yield {"op": "transition", "start": 800, "stop": 0, "step": 20}

        i += j


def _calc_forward_offset(offset, sensors):
    """Returns the offset for forward motion (go in a straight line)"""
    return 0.98 * offset + (0.28 * sensors["direction"] + 0.21 * sensors["omega"])


def parse_actions(actions):
    """Returns a coroutine that executes the actions sequentially"""
    ir = _actions_to_ir(actions)
    offset = 0
    sensors = yield (0, 0)
    for op in ir:
        if op["op"] == "transition":
            for val in range(op["start"], op["stop"], op["step"]):  # type: ignore
                offset = _calc_forward_offset(offset, sensors)
                sensors = yield (val - offset, val + offset)

        elif op["op"] == "hold-forward":
            for _ in range(500):  # FIXME: replace with distance
                offset = _calc_forward_offset(offset, sensors)
                sensors = yield (op["value"] - offset, op["value"] + offset)

        elif op["op"] == "reset":
            offset = 0
            sensors = yield "reset"


def run():
    # set up sensors
    gravity = calibrate(lambda: MOTION.acceleration)
    gyro_offset = calibrate(lambda: MOTION.gyro)
    gravity_unit = normalize(gravity)
    velocity = [integral() for _ in range(3)]
    position = [integral() for _ in range(3)]
    direction = integral()
    runner = parse_actions(ACTIONS)
    next(runner)  # initialize coroutine

    # set up motors
    for pin in MOTOR:
        pin.freq(1000)

    run_motor(0, 0)
    time.sleep(10)

    k = 0
    ticks = time.ticks_us()
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

        sensors = {
            "gyro": gyro,
            "omega": omega,
            "direction": direction(),
            "acceleration": acceleration,
            "velocity": eval_vec(velocity),
            "position": eval_vec(position),
        }

        try:
            action = runner.send(sensors)
        except StopIteration:
            run_motor(0, 0)
            break

        if action == "reset":
            for i in range(3):
                velocity[i](reset=True)
                position[i](reset=True)
            direction(reset=True)
        else:
            run_motor(*action)

        if False and k % 10 == 0:
            alpha = 0.85 * alpha + (2 * direction() + 1.5 * omega)
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
