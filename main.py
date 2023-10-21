from machine import Pin
import server

# pins
D5 = Pin(14, Pin.IN, Pin.PULL_UP)  # switch 1: app mode
D6 = Pin(12, Pin.IN, Pin.PULL_UP)  # swirch 4: modify program
LED = Pin(2, Pin.OUT)

# stop everything to flash on D2 == 0
if D6.value() != 0:
    if D5.value() == 0:
        # run file server
        LED.on()
        server.setup()
        server.run()
    else:
        # run app
        LED.off()
        import app

        app.run()
