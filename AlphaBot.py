"""
modified by mtc-20
"""
import RPi.GPIO as GPIO
import time

class AlphaBot(object):
    
    def __init__(self,in1=12,in2=13,ena=6,in3=20,in4=21,enb=26,s1=27,s2=22):
        self.IN1 = in1
        self.IN2 = in2
        self.IN3 = in3
        self.IN4 = in4
        self.ENA = ena
        self.ENB = enb
        self.S1 = 27
        self.S2 = 22

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.IN1,GPIO.OUT)
        GPIO.setup(self.IN2,GPIO.OUT)
        GPIO.setup(self.IN3,GPIO.OUT)
        GPIO.setup(self.IN4,GPIO.OUT)
        GPIO.setup(self.ENA,GPIO.OUT)
        GPIO.setup(self.ENB,GPIO.OUT)
        GPIO.setup(self.S1,GPIO.OUT)
        GPIO.setup(self.S2,GPIO.OUT)
        self.stop()
        self.PWMA = GPIO.PWM(self.ENA,500)
        self.PWMB = GPIO.PWM(self.ENB,500)
        self.PWMP = GPIO.PWM(self.S1,50)
        self.PWMT = GPIO.PWM(self.S2,50)
        self.PWMA.start(50)
        self.PWMB.start(50)
##        self.PWMP.start(10)
##        self.PWMT.start(7)
##        time.sleep(0.5)
        self.servo_switch(True)
        #self.PWMT.stop()
        #self.PWMP.stop()
        print('[Alpha_INFO]: Motors initialised')

    def forward(self):
        GPIO.output(self.IN1,GPIO.HIGH)
        GPIO.output(self.IN2,GPIO.LOW)
        GPIO.output(self.IN3,GPIO.LOW)
        GPIO.output(self.IN4,GPIO.HIGH)

    def stop(self):
        GPIO.output(self.IN1,GPIO.LOW)
        GPIO.output(self.IN2,GPIO.LOW)
        GPIO.output(self.IN3,GPIO.LOW)
        GPIO.output(self.IN4,GPIO.LOW)

    def backward(self):
        GPIO.output(self.IN1,GPIO.LOW)
        GPIO.output(self.IN2,GPIO.HIGH)
        GPIO.output(self.IN3,GPIO.HIGH)
        GPIO.output(self.IN4,GPIO.LOW)

    def left(self):
        GPIO.output(self.IN1,GPIO.LOW)
        GPIO.output(self.IN2,GPIO.LOW)
        GPIO.output(self.IN3,GPIO.LOW)
        GPIO.output(self.IN4,GPIO.HIGH)

    def right(self):
        GPIO.output(self.IN1,GPIO.HIGH)
        GPIO.output(self.IN2,GPIO.LOW)
        GPIO.output(self.IN3,GPIO.LOW)
        GPIO.output(self.IN4,GPIO.LOW)
        
    def setPWMA(self,value):
        self.PWMA.ChangeDutyCycle(value)

    def setPWMB(self,value):
        self.PWMB.ChangeDutyCycle(value)

    def setPWMP(self, angle):
        assert angle in range(0,181)
        value = (12.5/180.0)*angle + 3.5
        #print(value)
        self.PWMP.ChangeDutyCycle(value)
        #self.PWMP.start(value)
        print('Set Pan to {} deg'.format(angle))
        time.sleep(1)
        #self.PWMP.stop()

    def setPWMT(self, angle):
        assert angle in range(0,181)
        value = (7.5/180)*angle + 2.5
        #print(value)
        #self.PWMT.start(value)
        self.PWMT.ChangeDutyCycle(value)
        print('Set Tilt to {} deg'.format(angle))
        time.sleep(1)
        #self.PWMT.stop()

    def servo_switch(self, status):
        if status:
            self.PWMP.start(10)
            self.PWMT.start(7)
            time.sleep(2)
        else:
            print('[Alpha_INFO]: Switching off servos')
            self.PWMP.stop()
            self.PWMT.stop()
                    
        
    def setMotor(self, left, right):
        if((right >= 0) and (right <= 100)):
            GPIO.output(self.IN1,GPIO.HIGH)
            GPIO.output(self.IN2,GPIO.LOW)
            self.PWMA.ChangeDutyCycle(right)
        elif((right < 0) and (right >= -100)):
            GPIO.output(self.IN1,GPIO.LOW)
            GPIO.output(self.IN2,GPIO.HIGH)
            self.PWMA.ChangeDutyCycle(0 - right)
        if((left >= 0) and (left <= 100)):
            GPIO.output(self.IN3,GPIO.HIGH)
            GPIO.output(self.IN4,GPIO.LOW)
            self.PWMB.ChangeDutyCycle(left)
        elif((left < 0) and (left >= -100)):
            GPIO.output(self.IN3,GPIO.LOW)
            GPIO.output(self.IN4,GPIO.HIGH)
            self.PWMB.ChangeDutyCycle(0 - left)

    
if __name__ == '__main__':
        Ab = AlphaBot()
        time.sleep(2)
        Ab.stop()
        Ab.setPWMP(170)
        Ab.setPWMT(160)
##        Ab.servo_switch(False)
##        time.sleep(5)
##        print('Switching servos back on')
##        Ab.servo_switch(True)
        time.sleep(2)
        print('New pose')
        Ab.setPWMP(10)
        GPIO.cleanup()
