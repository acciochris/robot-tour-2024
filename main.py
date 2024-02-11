from machine import Pin

# pins
P = Pin(13, Pin.IN, Pin.PULL_UP)  # modify program

# stop everything to flash on D2 == 0
if P.value() != 0:
    import app

    app.run()
else:
    LED = Pin(2, Pin.OUT)
    LED.off()
