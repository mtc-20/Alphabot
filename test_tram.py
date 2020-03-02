#!/usr/bin/python
# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO
import time
from numpy import loadtxt
CS = 5
Clock = 25
Address = 24
DataOut = 23

class TRSensor(object):
  def __init__(self,numSensors = 5):
    self.numSensors = numSensors
    self.calibratedMin = [0] * self.numSensors
    self.calibratedMax = [1023] * self.numSensors
    self.last_value = 0
    
  """
  Reads the sensor values into an array. There *MUST* be space
  for as many values as there were sensors specified in the constructor.
  Example usage:
  unsigned int sensor_values[8];
  sensors.read(sensor_values);
  The values returned are a measure of the reflectance in abstract units,
  with higher values corresponding to lower reflectance (e.g. a black
  surface or a void).
  """
  def AnalogRead(self):
    value = [0,0,0,0,0,0]
    #Read Channel0~channel4 AD value
    for j in range(0,6):
      GPIO.output(CS, GPIO.LOW)
      for i in range(0,4):
        #sent 4-bit Address
        if(((j) >> (3 - i)) & 0x01):
          GPIO.output(Address,GPIO.HIGH)
        else:
          GPIO.output(Address,GPIO.LOW)
        #read MSB 4-bit data
        value[j] <<= 1
        if(GPIO.input(DataOut)):
          value[j] |= 0x01
        GPIO.output(Clock,GPIO.HIGH)
        GPIO.output(Clock,GPIO.LOW)
      for i in range(0,6):
        #read LSB 8-bit data
        value[j] <<= 1
        if(GPIO.input(DataOut)):
          value[j] |= 0x01
        GPIO.output(Clock,GPIO.HIGH)
        GPIO.output(Clock,GPIO.LOW)
      #no mean ,just delay
      for i in range(0,6):
        GPIO.output(Clock,GPIO.HIGH)
        GPIO.output(Clock,GPIO.LOW)
#     time.sleep(0.0001)
      GPIO.output(CS,GPIO.HIGH)
    return value[1:]
    
  """
  Reads the sensors 10 times and uses the results for
  calibration.  The sensor values are not returned; instead, the
  maximum and minimum values found over time are stored internally
  and used for the readCalibrated() method.
  """
  def calibrate(self):
    max_sensor_values = [0]*self.numSensors
    min_sensor_values = [0]*self.numSensors
    for j in range(0,10):
    
      sensor_values = self.AnalogRead();
      
      for i in range(0,self.numSensors):
      
        # set the max we found THIS time
        if((j == 0) or max_sensor_values[i] < sensor_values[i]):
          max_sensor_values[i] = sensor_values[i]

        # set the min we found THIS time
        if((j == 0) or min_sensor_values[i] > sensor_values[i]):
          min_sensor_values[i] = sensor_values[i]

    # record the min and max calibration values
    for i in range(0,self.numSensors):
      if(min_sensor_values[i] > self.calibratedMin[i]):
        self.calibratedMin[i] = min_sensor_values[i]
      if(max_sensor_values[i] < self.calibratedMax[i]):
        self.calibratedMax[i] = max_sensor_values[i]

  """
  Returns values calibrated to a value between 0 and 1000, where
  0 corresponds to the minimum value read by calibrate() and 1000
  corresponds to the maximum value.  Calibration values are
  stored separately for each sensor, so that differences in the
  sensors are accounted for automatically.
  """
  def readCalibrated(self):
    value = 0
    #read the needed values
    sensor_values = self.AnalogRead();

    for i in range (0,self.numSensors):

      denominator = self.calibratedMax[i] - self.calibratedMin[i]

      if(denominator != 0):
        value = (sensor_values[i] - self.calibratedMin[i])* 1000 / denominator
        
      if(value < 0):
        value = 0
      elif(value > 1000):
        value = 1000
        
      sensor_values[i] = value
    
    #print("readCalibrated",sensor_values)
    return sensor_values
      
  """
  Operates the same as read calibrated, but also returns an
  estimated position of the robot with respect to a line. The
  estimate is made using a weighted average of the sensor indices
  multiplied by 1000, so that a return value of 0 indicates that
  the line is directly below sensor 0, a return value of 1000
  indicates that the line is directly below sensor 1, 2000
  indicates that it's below sensor 2000, etc.  Intermediate
  values indicate that the line is between two sensors.  The
  formula is:

     0*value0 + 1000*value1 + 2000*value2 + ...
     --------------------------------------------
       value0  +  value1  +  value2 + ...

  By default, this function assumes a dark line (high values)
  surrounded by white (low values).  If your line is light on
  black, set the optional second argument white_line to true.  In
  this case, each sensor value will be replaced by (1000-value)
  before the averaging.
  """
  def readLine(self, white_line = 0):

    sensor_values = self.readCalibrated()
    avg = 0
    sum = 0
    on_line = 0
    for i in range(0,self.numSensors):
      value = sensor_values[i]
      if(white_line):
        value = 1000-value
      # keep track of whether we see the line at all
      if(value > 200):
        on_line = 1
        
      # only average in values that are above a noise threshold
      if(value > 50):
        avg += value * (i * 1000);  # this is for the weighted total,
        sum += value;                  #this is for the denominator 

    if(on_line != 1):
      # If it last read to the left of center, return 0.
      if(self.last_value < (self.numSensors - 1)*1000/2):
        #print("left")
        return 0;
  
      # If it last read to the right of center, return the max.
      else:
        #print("right")
        return (self.numSensors - 1)*1000

    self.last_value = avg/sum
    
    return self.last_value


import cv2, Queue, threading, time

# bufferless VideoCapture
class VideoCapture:

  def __init__(self, name):
    self.cap = cv2.VideoCapture(name)
    self.q = Queue.Queue()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      if not self.q.empty():
        try:
          self.q.get_nowait()   # discard previous (unprocessed) frame
        except Queue.Empty:
          pass
      self.q.put(frame)

  def read(self):
    return self.q.get()

def turnAround():
    global Ab
    time.sleep(1.5)
    Ab.stop()
    time.sleep(2)
    print('Executing turn around')
    Ab.setPWMA(0)
    Ab.setPWMB(maximum)
    Ab.forward()
    time.sleep(2)
    Ab.stop()
    Ab.backward()

  
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(Clock,GPIO.OUT)
GPIO.setup(Address,GPIO.OUT)
GPIO.setup(CS,GPIO.OUT)
GPIO.setup(DataOut,GPIO.IN,GPIO.PUD_UP)


maximum = 35;

if __name__ == '__main__':

  from AlphaBot import AlphaBot
  from hsv_tracker import hsv_trackbar

  minHSV = (112, 57, 26) #(30,7,70)
  maxHSV = (175,100,185)
  #[minHSV, maxHSV] = hsv_trackbar()
  #[minHSV, maxHSV] = loadtxt('blue.out').astype(int)
  print(minHSV, maxHSV)
  maximum = 35;
  integral = 0;
  last_proportional = 0
  
  TR = TRSensor()
  Ab = AlphaBot()
  Ab.stop()
  cap = VideoCapture(1)
  print("Tram Example")
  time.sleep(0.5)
  for i in range(0,250):
    TR.calibrate()
    print (i)
  print(TR.calibratedMin)
  print(TR.calibratedMax)
  time.sleep(1) 
  Ab.backward()
  while cap.cap.isOpened():
    position = TR.readLine(1)
    #print(position)
    
    # The "proportional" term should be 0 when we are on the line.
    proportional = position - 2000
    
    # Compute the derivative (change) and integral (sum) of the position.
    derivative = proportional - last_proportional
    integral += proportional
    
    # Remember the last position.
    last_proportional = proportional
  
    '''
    // Compute the difference between the two motor power settings,
    // m1 - m2.  If this is a positive number the robot will turn
    // to the right.  If it is a negative number, the robot will
    // turn to the left, and the magnitude of the number determines
    // the sharpness of the turn.  You can adjust the constants by which
    // the proportional, integral, and derivative terms are multiplied to
    // improve performance.
    '''
    power_difference = proportional/25 + derivative/100 #+ integral/1000;  

    if (power_difference > maximum):
      power_difference = maximum
    if (power_difference < - maximum):
      power_difference = - maximum
    #print(position,power_difference)
    if (power_difference < 0):
      Ab.setPWMB(maximum + power_difference)
      Ab.setPWMA(maximum);
    else:
      Ab.setPWMB(maximum);
      Ab.setPWMA(maximum - power_difference)

    # Check for end of line
    frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsvMask = cv2.inRange(hsv, minHSV, maxHSV)
    result = cv2.bitwise_and(frame, frame, mask=hsvMask)
    cnts = cv2.findContours(hsvMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:1]
    for c in cnts:
                        area = cv2.contourArea(c)/100
                        
                        if area > 320:
                                print('area:', area)
                                cv2.drawContours(frame, [c], -1,(0,20,200), -1)
                                M = cv2.moments(c)
                                if (M['m00']!=0):
                                        
                                        turnAround()
                                        print('Resuming line follower')
                                        
        
##    cv2.imshow('HSV', result)
##    k = cv2.waitKey(1)
##    if k%256==27:
##      print('Break')
##      break
  cv2.destroyAllWindows()
  cap.cap.release()

