from machine import I2C, Pin
from ssd1306 import SSD1306_I2C

DISPLAY_I2C = I2C(sda=Pin(4), scl=Pin(5))
DISPLAY = SSD1306_I2C(128, 64, DISPLAY_I2C)


def update():
    DISPLAY.text("Hello, World!", 0, 0, 1)
    DISPLAY.show()
