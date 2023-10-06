#import GPIO and time
import RPi.GPIO as GPIO
import time

# set GPIO numbering mode and define output pins
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)

# cycle those relays
try:
    
    GPIO.output(21, True)
    time.sleep(20)
    GPIO.output(21, False)
    GPIO.output(20, True)
    time.sleep(13)
    GPIO.output(20, False)

finally:
    # cleanup the GPIO before finishing :)
    GPIO.cleanup()

