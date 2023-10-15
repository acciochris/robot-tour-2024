from machine import Pin
import server

# pins
D1 = Pin(5, Pin.IN, Pin.PULL_UP)
D2 = Pin(4, Pin.IN, Pin.PULL_UP)
LED = Pin(2, Pin.OUT)

# stop everything to flash on D2 == 0
if D2.value() != 0:
    if D1.value() == 0:
        # run file server
        LED.on()
        server.setup()
        server.run()
    else:
        # run app
        LED.off()
        import app

        app.run()
