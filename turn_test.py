#!/usr/bin/python
# -*- coding:utf-8 -*-

import RPi.GPIO as GPIO
from AlphaBot import AlphaBot
import time
import cv2
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


CS = 5
Clock = 25
Address = 24
DataOut = 23


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(Clock,GPIO.OUT)
GPIO.setup(Address,GPIO.OUT)
GPIO.setup(CS,GPIO.OUT)
GPIO.setup(DataOut,GPIO.IN,GPIO.PUD_UP)

def turnAround():
    global Ab
    Ab.stop()
    time.sleep(2)
    print('Executing turn around')
    Ab.setPWMA(0)
    Ab.setPWMB(35)
    Ab.forward()
    time.sleep(2)
    Ab.stop()
    

if __name__ == '__main__':
    minHSV = (77,70,70)
    maxHSV = (179,255,255)
    Ab = AlphaBot()
    Ab.stop()
##    Ab.forward()
##    time.sleep(2)
##    turnAround()
    cap = VideoCapture(0)
##    cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)
    while cap.cap.isOpened():
        frame = cap.read()
##        if not ret:
##            break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsvMask = cv2.inRange(hsv, minHSV, maxHSV)
        cnts = cv2.findContours(hsvMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:3]
        for c in cnts:
            area = cv2.contourArea(c)
            if area > 15000:
                cv2.drawContours(frame, [c], -1,(0,20,200), -1)
                M = cv2.moments(c)
                if (M['m00']!=0):
                    cX = int(M['m10']/M['m00'])
                    cY = int(M['m01']/M['m00'])
                    cv2.circle(frame, (cX,cY),3,(100,0,0),-1)
                    turnAround()
                    time.sleep(2)
                    print('execution complete')
        cv2.imshow('result', frame)
        k = cv2.waitKey(1)
        if k%256 == 27:
            break
    cv2.destroyAllWindows()
    cap.cap.release()

        
    
