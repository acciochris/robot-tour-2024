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


ACTIONS = "FRFFRRRFFRFFRRRFFRRRFFRFFRRFFRFFRRRFFRFFRRFFRRRFFRRRFFRFFRFF"

DEG_TO_RAD = math.pi / 180
DEG_90 = 90 * DEG_TO_RAD

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


MOTION.ak8963._offset = (37.691, 33.0422, -18.6492)
MOTION.ak8963._scale = (1.04683, 0.99886, 0.95823)

# LIDAR = VL53L0X(I2C_CONFIG)

MOTOR = [
    PWM(Pin(0)),
    PWM(Pin(2)),
    PWM(Pin(13)),
    PWM(Pin(15)),
]

target_direction = 0


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


def calibrate(*funcs, n=50):
    offsets = [[0.0, 0.0, 0.0] for _ in range(len(funcs))]
    for _ in range(n):
        for i, func in enumerate(funcs):
            x, y, z = func()
            offsets[i][0] += x
            offsets[i][1] += y
            offsets[i][2] += z
        time.sleep_ms(20)
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


def angle_diff(ang1, ang2):
    """Returns the difference between two angles, taking into account the wrap-around"""
    return abs((ang1 - ang2 + math.pi) % (2 * math.pi) - math.pi)


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
            yield {"op": "transition", "start": 500, "stop": 800, "step": 15}
            yield {"op": "forward", "value": 800, "distance": j * 0.23, "smooth": True}
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
    return 0.85 * offset + (150 * sensors["direction"] + 150 * sensors["omega"])


def parse_actions(actions):
    """Returns a coroutine that executes the actions sequentially"""
    global target_direction

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

                if sensors["lidar"] <= 500:
                    dist_to_go = min(dist_to_go, sensors["lidar"] - 250)

                if dist_to_go <= 0:
                    break

                if op.get("smooth", False):
                    val = 500 + 7500 * dist_to_go**2
                    val = min(val, op["value"])
                else:
                    val = op["value"]

                offset = _calc_forward_offset(offset, sensors)
                sensors = yield (val - offset, val + offset)

        elif op["op"] == "turn":
            if op["direction"] == "cw":
                target_direction = target_direction - op["angle"] * DEG_TO_RAD
            else:
                target_direction = target_direction + op["angle"] * DEG_TO_RAD

            for _ in range(500):  # timeout after 5 seconds
                # show_message("Turning...")(1.24303, 0.93657, 0.88669)
                current_dir = abs(sensors["direction"])
                angle_to_go = op["angle"] * DEG_TO_RAD - current_dir  # type: ignore

                # start using magnetic sensor
                if angle_to_go <= 40 * DEG_TO_RAD:
                    angle_to_go = angle_diff(target_direction, sensors["magdir"])
                    show_message(str(sensors["magdir"]))

                if angle_to_go <= 16 * DEG_TO_RAD:
                    break

                if op.get("smooth", False):
                    val = 400 + 600 * angle_to_go**2
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
    global target_direction

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
    while True:
        lidar = LIDAR.ping()
        show_message(str(lidar))

        if lidar < 40:
            break
        time.sleep_ms(500)

    for m in range(3, 0, -1):  # countdown
        show_message(str(m))
        time.sleep(1)

    show_message("Running...")
    k = 0
    acceleration = [0, 0, 0]
    magnetic = MOTION.magnetic
    target_direction = math.atan2(magnetic[1], magnetic[0])
    ticks = time.ticks_us()
    while True:
        acceleration = [
            a_prev * 0.5 + a * 0.5
            for a_prev, a in zip(
                acceleration, with_offset(MOTION.acceleration, gravity)
            )
        ]
        gyro = with_offset(MOTION.gyro, gyro_offset)
        magnetic = MOTION.magnetic
        lidar = LIDAR.ping()
        current_ticks = time.ticks_us()
        step = time.ticks_diff(current_ticks, ticks) / 1_000_000

        # integrate acceleration
        for i in range(3):
            position[i](velocity[i](acceleration[i], step), step)

        # calculate and integrate angular velocity
        omega = dot(gyro, gravity_unit)
        direction(omega, step)
        magdir = math.atan2(magnetic[1], magnetic[0])

        sensors = {
            "gyro": gyro,
            "omega": omega,
            "direction": direction(),
            "acceleration": acceleration,
            "velocity": eval_vec(velocity),
            "position": eval_vec(position),
            "lidar": lidar,
            "magdir": magdir,
        }

        try:
            action = runner.send(sensors)
        except StopIteration:
            show_message("Done!")
            action = (0, 0)

        if action == "reset":
            # calibrate sensors
            time.sleep(1)  # wait for the robot to stop moving
            gravity, gyro_offset = calibrate(
                lambda: MOTION.acceleration,
                lambda: MOTION.gyro,
            )
            gravity_unit = normalize(gravity)

            # reset
            for i in range(3):
                acceleration = [0, 0, 0]
                velocity[i](reset=True)
                position[i](reset=True)
            direction(reset=True)
        else:
            print(*action)
            run_motor(*action)

        k += 1
        ticks = current_ticks
        time.sleep_ms(10)
