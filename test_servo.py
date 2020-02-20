import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

def set_pos(x):
    assert x in range(0,181)
    GPIO.setup(22, GPIO.OUT)
    pwm = GPIO.PWM(22, 50)
    pwm.start(7)
    print('Initializing')
    time.sleep(2)
    cycle = (7.5/180)*x + 2.5
    print(cycle)
    pwm.ChangeDutyCycle(cycle)
    time.sleep(0.3)
    pwm.stop()
    GPIO.cleanup()

# Theoretically for 50Hz, extremes are 5% and 10% of duty cycle
try:
    # Tilt
    GPIO.setup(27, GPIO.OUT)
    pwm = GPIO.PWM(27, 50)
    pwm.start(10)
    time.sleep(3)
    pwm.ChangeDutyCycle(15.2) #full down
    print('Left')
    time.sleep(3)
    print('Goinf home')
    pwm.ChangeDutyCycle(10)
    time.sleep(2)
    print('Right')
    pwm.ChangeDutyCycle(3) # full up
    time.sleep(2)
    pwm.stop()
    print('STOP')
    
    #pwm.ChangeDutyCycle(10)
    #pwm.stop()
    #set_pos(170)

    GPIO.cleanup()

except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()
