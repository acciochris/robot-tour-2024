# Note:
# To minify, execute `pyminify app.py -o app.min.py --remove-literal-statements --rename-globals --preserve-globals run`

# Micropython
from machine import I2C, Pin, PWM
import time
import math

# Sensors
from mpu9250 import MPU9250
from vl53l0x import VL53L0X

# Display
from ssd1306 import SSD1306_I2C


ACTIONS = "FLFRF"

DEG_TO_RAD = math.pi / 180

I2C_CONFIG = I2C(sda=Pin(4), scl=Pin(5))
DISPLAY = SSD1306_I2C(128, 64, I2C_CONFIG)


def show_message(msg: str):
    DISPLAY.fill(0)
    DISPLAY.text(msg, 0, 0, 1)
    DISPLAY.show()


show_message("Initializing...")
try:
    MOTION = MPU9250(I2C_CONFIG)
except Exception:
    show_message("MPU9250")
    raise
try:
    LIDAR = VL53L0X(I2C_CONFIG)
except Exception:
    show_message("LIDAR")
    raise


# LIDAR = VL53L0X(I2C_CONFIG)

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


def calibrate(*funcs, n=20):
    offsets = [[0.0, 0.0, 0.0] for _ in range(len(funcs))]
    for _ in range(n):
        for i, func in enumerate(funcs):
            x, y, z = func()
            offsets[i][0] += x
            offsets[i][1] += y
            offsets[i][2] += z
        time.sleep_ms(10)
    for offset in offsets:
        offset[0] /= n
        offset[1] /= n
        offset[2] /= n
    return offsets


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
            yield {"op": "transition", "start": 300, "stop": 800, "step": 15}
            yield {"op": "forward", "value": 800, "distance": j / 4, "smooth": True}
            yield {"op": "stop"}
        elif action == "T":
            yield {"op": "reset"}
            yield {"op": "forward", "value": 300, "distance": 1}
        elif action == "L":
            yield {"op": "reset"}
            yield {
                "op": "turn",
                "direction": "ccw",
                "value": 800,
                "angle": 10 * j,
            }  # get the turn started
            yield {
                "op": "turn",
                "direction": "ccw",
                "value": 600,
                "angle": 90 * j,
                "smooth": True,
            }  # turn the rest of the way
            yield {"op": "stop"}
        elif action == "R":
            yield {"op": "reset"}
            yield {"op": "turn", "direction": "cw", "value": 800, "angle": 10 * j}
            yield {
                "op": "turn",
                "direction": "cw",
                "value": 600,
                "angle": 90 * j,
                "smooth": True,
            }
            yield {"op": "stop"}
        else:
            raise ValueError("Invalid action")

        i += j


def _calc_forward_offset(offset, sensors):
    """Returns the offset for forward motion (go in a straight line)"""
    return 0.98 * offset + (20 * sensors["direction"] + 20 * sensors["omega"])


def _calc_angle_lidar(sensors):
    r"""Returns the angle to the nearest obstacle

    $$
    \theta = \arctan\left(\frac{\frac{dL}{d\theta} - x}{L + y}\right)
    $$
    """

    dLdtheta = (
        sensors["dL/dt"] / sensors["omega"]
    )  # dL/dtheta = (dL/dt) / (dtheta/dt) (in mm)
    return math.atan((dLdtheta - 56) / (sensors["lidar"] + 120))


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

        elif op["op"] == "forward":
            for _ in range(500):  # timeout after 5 seconds
                current_pos = norm(sensors["position"])
                dist_to_go = op["distance"] - current_pos  # type: ignore
                if dist_to_go <= 0:
                    break

                if op.get("smooth", False):
                    val = 300 + 7500 * dist_to_go**2
                    val = min(val, op["value"])
                else:
                    val = op["value"]

                offset = _calc_forward_offset(offset, sensors)
                sensors = yield (val - offset, val + offset)

        elif op["op"] == "turn":
            lidar_coef = -1 if op["direction"] == "ccw" else 1
            for _ in range(500):  # timeout after 5 seconds
                current_dir = abs(sensors["direction"])
                angle_to_go = op["angle"] * DEG_TO_RAD - current_dir  # type: ignore

                if sensors["lidar"] <= 350 and angle_to_go <= math.pi / 6:  # start using lidar when close to target
                    angle_to_go = _calc_angle_lidar(sensors) * lidar_coef

                if angle_to_go <= 0:
                    break

                if op.get("smooth", False):
                    val = 350 + 600 * angle_to_go**2
                    val = min(val, op["value"])
                else:
                    val = op["value"]

                if op["direction"] == "ccw":
                    sensors = yield (val, -val)  # type: ignore
                elif op["direction"] == "cw":
                    sensors = yield (-val, val)  # type: ignore

        elif op["op"] == "reset":
            offset = 0
            sensors = yield "reset"

        elif op["op"] == "stop":
            sensors = yield (0, 0)


def run():
    gravity, gyro_offset = calibrate(
        lambda: MOTION.acceleration,
        lambda: MOTION.gyro,
    )
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
    show_message("Motor")
    time.sleep(10)

    k = 0
    dLdt = 0
    last_lidar = LIDAR.ping()
    ticks = time.ticks_us()
    last_lidar_ticks = ticks
    while True:
        acceleration = with_offset(MOTION.acceleration, gravity)
        gyro = with_offset(MOTION.gyro, gyro_offset)
        lidar = LIDAR.ping()
        current_ticks = time.ticks_us()
        step = time.ticks_diff(current_ticks, ticks) / 1_000_000

        # integrate acceleration
        for i in range(3):
            position[i](velocity[i](acceleration[i], step), step)

        # calculate and integrate angular velocity
        omega = dot(gyro, gravity_unit)
        direction(omega, step)

        # dL/dt
        if k % 10 == 9:  # reduce frequency to avoid the effect of noise
            dLdt = (lidar - last_lidar) / (
                time.ticks_diff(current_ticks, last_lidar_ticks) / 1_000_000
            )
            last_lidar = lidar
            last_lidar_ticks = current_ticks

        sensors = {
            "gyro": gyro,
            "omega": omega,
            "direction": direction(),
            "acceleration": acceleration,
            "velocity": eval_vec(velocity),
            "position": eval_vec(position),
            "lidar": lidar,
            "dL/dt": dLdt,
        }

        try:
            action = runner.send(sensors)
        except StopIteration:
            action = (0, 0)

        if action == "reset":
            # calibrate sensors
            time.sleep_ms(500)
            gravity, gyro_offset = calibrate(
                lambda: MOTION.acceleration,
                lambda: MOTION.gyro,
            )
            gravity_unit = normalize(gravity)

            # reset integrals
            for i in range(3):
                velocity[i](reset=True)
                position[i](reset=True)
            direction(reset=True)
        else:
            print(*action)
            # run_motor(*action)

        if False and k % 10 == 0:
            alpha = 0.85 * alpha + (2 * direction() + 1.5 * omega)
            run_motor(800 - alpha, 800 + alpha)
            DISPLAY.fill(0)
            DISPLAY.text(str(alpha), 0, 0, 1)
            DISPLAY.show()
            # print(alpha)

        if k % 20 == 0 and False:
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

        if k % 20 == 0:
            DISPLAY.fill(0)
            DISPLAY.text(str(lidar), 0, 0, 1)
            DISPLAY.text(str(int(sensors["direction"] / DEG_TO_RAD)), 0, 8, 1)
            DISPLAY.text(str(int(_calc_angle_lidar(sensors) / DEG_TO_RAD)), 0, 16, 1)
            DISPLAY.show()

        k += 1
        ticks = current_ticks
        time.sleep_ms(10)
