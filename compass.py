import math
import time

from mpu9250 import MPU9250
from ak8963 import AK8963
from ssd1306 import SSD1306_I2C
from machine import I2C, Pin

I2C_CONFIG = I2C(sda=Pin(4), scl=Pin(5))

def run():
    _mpu9250 = MPU9250(I2C_CONFIG)
    ak8963 = AK8963(I2C_CONFIG, offset=(36.3639, -10.9547, 1.79649), scale=(1.0784, 1.03296, 0.905292))
    oled = SSD1306_I2C(128, 64, I2C_CONFIG)
    
    while True:
        x, y, _ = ak8963.magnetic
        oled.fill(0)
        oled.text(f"{math.atan2(y, x) * 180 / math.pi:.2f}", 0, 0, 1)
        oled.show()
        time.sleep(.2)
