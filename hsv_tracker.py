import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray



def nothing(x):
    pass

def hsv_trackbar(filename='test'):
    save_file = '{}.out'.format(filename)
    cap = cv2.VideoCapture(0)
    cv2.namedWindow("Trackbars")
    cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
    cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
    cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break 
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")
        lower_hsv = np.array([l_h, l_s, l_v])
        upper_hsv = np.array([u_h, u_s, u_v])
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        result = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow("qframe", frame)
        cv2.imshow("mask", mask)
        cv2.imshow("result", result)
        key = cv2.waitKey(1)
        if key%256 == 27:
            break
        if key%256 == 32:
            print('lowHSV:', lower_hsv)
            print('highHSV:', upper_hsv)
        
    #np.savetxt(save_file, [lower_hsv, upper_hsv])
##    cap.release()
    cv2.destroyAllWindows()
##    np.savetxt(file, [lower_hsv, upper_hsv])

    
    return np.array(lower_hsv), np.array(upper_hsv)


##def hsv_trackbar(filename='test'):
##    save_file = '{}.out'.format(filename)
##    cap = PiCamera(sensor_mode=6)
##    cap.resolution = (640, 480)
##    rawCapture = PiRGBArray(cap)
##    cv2.namedWindow("Trackbars")
##    cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
##    cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
##    cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
##    cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
##    cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
##    cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)
##    for image in cap.capture_continuous(rawCapture, format="bgr", use_video_port=True):
##        frame = image.array
##        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
##        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
##        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
##        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
##        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
##        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
##        u_v = cv2.getTrackbarPos("U - V", "Trackbars")
##        lower_hsv = np.array([l_h, l_s, l_v])
##        upper_hsv = np.array([u_h, u_s, u_v])
##        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
##        result = cv2.bitwise_and(frame, frame, mask=mask)
##        cv2.imshow("qframe", frame)
##        cv2.imshow("mask", mask)
##        cv2.imshow("result", result)
##        key = cv2.waitKey(1)
##        if key%256 == 27:
##            break
##        if key%256 == 32:
##            print('lowHSV:', lower_hsv)
##            print('highHSV:', upper_hsv)
##        rawCapture.truncate(0)
##    np.savetxt(save_file, [lower_hsv, upper_hsv])
####    cap.release()
##    cv2.destroyAllWindows()
####    np.savetxt(file, [lower_hsv, upper_hsv])
##
##    
##    return np.array(lower_hsv), np.array(upper_hsv)
##
##
##
##def hsv_fromimage(file):
##    cv2.namedWindow("Trackbars")
##    cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
##    cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
##    cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
##    cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
##    cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
##    cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)
##    while True:
##        frame = cv2.imread(file)
##        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
##        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
##        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
##        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
##        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
##        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
##        u_v = cv2.getTrackbarPos("U - V", "Trackbars")
##        lower_hsv = np.array([l_h, l_s, l_v])
##        upper_hsv = np.array([u_h, u_s, u_v])
##        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
##        result = cv2.bitwise_and(frame, frame, mask=mask)
##        cv2.imshow("qframe", frame)
##        cv2.imshow("mask", mask)
##        cv2.imshow("result", result)
##        key = cv2.waitKey(1)
##        if key%256 == 27:
##            break
##        if key%256 == 32:
##            print('lowHSV:', lower_hsv)
##            print('highHSV:', upper_hsv)
##    cv2.destroyAllWindows()
##
##    
##    return np.array(lower_hsv), np.array(upper_hsv)

if __name__=='__main__':
    minHSV, maxHSV = hsv_trackbar()
##    minHSV, maxHSV = hsv_fromimage('dataset_4.jpg')
##    image = cv2.imread('dataset_4.jpg')
##    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
##    hsvMask = cv2.inRange(hsv, minHSV, maxHSV)
##    cnts = cv2.findContours(hsvMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
##    print cnts
##    area = cv2.contourArea(cnts[0])
##    print area
##    cap = PiCamera(sensor_mode=6)
##    cap.resolution = (640, 480)
##    rawCapture = PiRGBArray(cap)
##    for image in cap.capture_continuous(rawCapture, format="bgr", use_video_port=True):
##        frame = image.array
##        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
##        hsvMask = cv2.inRange(hsv, minHSV, maxHSV)
##        cnts = cv2.findContours(hsvMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
##        cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:3]
##        for c in cnts:
##            cv2.drawContours(frame, c,-1, (0,0, 50), 1)
##            area = cv2.contourArea(np.array(c)) 
##            print(area)
##    ##        print c
##        cv2.imshow('result', frame)
##        key = cv2.waitKey(1)
##        if key%256 == 27:
##            break
##        rawCapture.truncate(0)
##    cv2.destroyAllWindows()
