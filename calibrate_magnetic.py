from machine import I2C, Pin
from mpu9250 import MPU9250
from ssd1306 import SSD1306_I2C

I2C_CONFIG = I2C(sda=Pin(4), scl=Pin(5))
DISPLAY = SSD1306_I2C(128, 64, I2C_CONFIG)

def run():
    ak8963 = MPU9250(I2C_CONFIG).ak8963
    DISPLAY.fill(0)
    DISPLAY.text("Calibrating", 0, 0, 1)
    DISPLAY.show()
    offset, scale = ak8963.calibrate()
    with open("magnetic.txt", "w") as f:
        f.write(repr(offset) + "\n")
        f.write(repr(scale) + "\n")
