# Note:
# To minify, execute `pyminify app.py -o app.min.py --remove-literal-statements --rename-globals --preserve-globals run`

# Micropython
from machine import SoftI2C, Pin, PWM
import time
import math
import machine

# Sensors
from mpu9250 import MPU9250
from vl53l0x import VL53L0X

# Display
from ssd1306 import SSD1306_I2C


ACTIONS = "^RFFLFFLFFRFFRFFLFFLFFFFF$"

DEG_TO_RAD = math.pi / 180

I2C_CONFIG = SoftI2C(sda=Pin(21), scl=Pin(22))
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
    PWM(Pin(16, drive=Pin.DRIVE_3)),
    PWM(Pin(17, drive=Pin.DRIVE_3)),
    PWM(Pin(18, drive=Pin.DRIVE_3)),
    PWM(Pin(19, drive=Pin.DRIVE_3)),
]

### Encoders
ENC1 = Pin(32, Pin.IN)  # right
ENC2 = Pin(33, Pin.IN)  # left
enc1 = 0
enc2 = 0

def handler1(_pin):
    global enc1
    enc1 += 1

def handler2(_pin):
    global enc2
    enc2 += 1


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


def calibrate(*funcs, n=60):
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
            # yield {"op": "transition", "start": 500, "stop": 800, "step": 15}
            yield {"op": "forward", "value": 800, "distance": j * 0.25, "smooth": True}
            yield {"op": "stop"}
        elif action == "B":
            yield {"op": "reset"}
            yield {"op": "forward", "value": -800, "distance": j * 0.25, "smooth": True}
            yield {"op": "stop"}
        elif action == "^":
            yield {"op": "reset"}
            yield {"op": "forward", "value": 800, "distance": 0.33, "smooth": True}
            yield {"op": "stop"}
        elif action == "$":
            yield {"op": "reset"}
            yield {"op": "forward", "value": 800, "distance": 0.17, "smooth": True}
            yield {"op": "stop"}
        elif action == "L":
            yield {"op": "reset"}
            # yield {
            #     "op": "turn",
            #     "direction": "ccw",
            #     "value": 1000,
            #     "angle": 10 * j,
            # }  # get the turn started
            yield {
                "op": "turn",
                "direction": "ccw",
                "value": 800,
                "angle": 90 * j,
                "smooth": True,
            }  # turn the rest of the way
            yield {"op": "stop"}
        elif action == "R":
            yield {"op": "reset"}
            # yield {"op": "turn", "direction": "cw", "value": 1000, "angle": 10 * j}
            yield {
                "op": "turn",
                "direction": "cw",
                "value": 800,
                "angle": 90 * j,
                "smooth": True,
            }
            yield {"op": "stop"}
        elif action == "P":
            # pause
            pass
        else:
            raise ValueError("Invalid action")

        i += j


def _calc_forward_offset(offset, sensors):
    """Returns the offset for forward motion (go in a straight line)"""
    return 0.9 * offset + (100 * sensors["direction"] + 100 * sensors["omega"])
    # encoder_delta = sensors["encoder"][0] - sensors["encoder"][1]
    # return offset * 0.95 + encoder_delta * 4


def parse_actions(actions):
    """Returns a coroutine that executes the actions sequentially"""
    ir = _actions_to_ir(actions)
    offset = 0
    sensors = yield (0, 0)
    for op in ir:
        # if op["op"] == "transition":
        #     for val in range(op["start"], op["stop"], op["step"]):  # type: ignore
        #         offset = _calc_forward_offset(offset, sensors)
        #         sensors = yield (val - offset, val + offset)

        if op["op"] == "forward":
            show_message(f"fw {op['value']:d}")
            sign = 1 if op["value"] >= 0 else -1  # type: ignore
            val = 0
            for _ in range(500):  # timeout after 5 seconds
                current_pos = (
                    sensors["encoder"][0] + sensors["encoder"][1]
                ) / 367
                dist_to_go = op["distance"] - current_pos  # type: ignore

                if sign == 1 and sensors["lidar"] <= 500:
                    dist_to_go = min(dist_to_go, (sensors["lidar"] - 200) / 1000)

                if dist_to_go <= .01:
                    # spike in the opposite direction to stop
                    yield (-val * sign / 2, -val * sign / 2)  # type: ignore
                    break

                if op.get("smooth", False):
                    val = 500 + 30_000 * dist_to_go**2
                    val = min(val, op["value"] * sign)
                else:
                    val = op["value"] * sign

                offset = _calc_forward_offset(offset, sensors)
                sensors = yield (val * sign - offset, val * sign + offset)

        elif op["op"] == "turn":
            show_message(op["direction"])
            val = 0
            for _ in range(500):  # timeout after 5 seconds
                # show_message("Turning...")
                current_dir = abs(sensors["direction"])
                angle_to_go = op["angle"] * DEG_TO_RAD - current_dir  # type: ignore

                if angle_to_go <= 10 * DEG_TO_RAD and abs(sensors["omega"]) < .01:
                    break

                if angle_to_go <= 7 * DEG_TO_RAD:
                    # stop things by spiking in the opposite direction
                    if op["direction"] == "ccw":
                        sensors = yield (-val / 2, val / 2)  # type: ignore
                    elif op["direction"] == "cw":
                        sensors = yield (val / 2, -val / 2)  # type: ignore
                    break

                if op.get("smooth", False):
                    if op["direction"] == "cw":
                        val = 520 + 600 * angle_to_go**2
                    else:
                        val = 520 + 600 * angle_to_go**2
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
            show_message("stop")
            sensors = yield (0, 0)


def run():
    global enc1, enc2

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

        if lidar < 50:
            break
        time.sleep_ms(500)

    for m in range(3, 0, -1):  # countdown
        show_message(str(m))
        time.sleep(1)

    show_message("Running...")
    ENC1.irq(handler1, Pin.IRQ_RISING | Pin.IRQ_FALLING)
    ENC2.irq(handler2, Pin.IRQ_RISING | Pin.IRQ_FALLING)
    k = 0
    acceleration = [0, 0, 0]
    ticks = time.ticks_us()
    while True:
        acceleration = [
            a_prev * .5 + a * .5
            for a_prev, a in
            zip(acceleration, with_offset(MOTION.acceleration, gravity))
        ]
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

        sensors = {
            "gyro": gyro,
            "omega": omega,
            "direction": direction(),
            "acceleration": acceleration,
            "velocity": eval_vec(velocity),
            "position": eval_vec(position),
            "lidar": lidar,
            "encoder": (enc1, enc2),
        }

        try:
            action = runner.send(sensors)
        except StopIteration:
            show_message("Done!")
            action = (0, 0)

        if action == "reset":
            # calibrate sensors
            time.sleep(.7)  # wait for the robot to stop moving
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

            # reset encoders
            irq = machine.disable_irq()
            enc1 = 0
            enc2 = 0
            machine.enable_irq(irq)
        else:
            # show_message(f"{action[0]:.1f}, {action[1]:.1f}")
            run_motor(*action)

        k += 1
        ticks = current_ticks
        time.sleep_ms(10)
