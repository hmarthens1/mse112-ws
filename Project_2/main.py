#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/mse112-ws/MasterPi/')
import cv2
import time
import Camera
import threading
import yaml_handle
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Sonar as Sonar
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
import numpy as np
import math

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()
HWSONAR = Sonar.Sonar() #Ultrasonic Sensor

__target_shape = ('circle', 'triangle', 'square')

def setTargetShape(target_shape):
    global __target_shape
    print("SHAPE", target_shape)
    __target_shape = target_shape
    return (True, ())

# Find the contour with the largest area
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours: #Go through all contours
        contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate the contour area
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:  # The contour of the largest area is valid only when the area is greater than 300 to filter out interference
                area_max_contour = c

    return area_max_contour, contour_area_max  # Return the largest contour

# The closing angle of the gripper when gripping
servo1 = 1500

# Initial position
def initMove():
    Board.setPWMServoPulse(1, 2500, 300)
    time.sleep(0.5)
    Board.setPWMServoPulse(3, 800, 500) 
    time.sleep(0.5)
    Board.setPWMServoPulse(4, 2000, 500) 
    time.sleep(0.5)
    Board.setPWMServoPulse(5, 2100, 500) 
    time.sleep(1)
    Board.setPWMServoPulse(6, 1500, 500)
    time.sleep(1)
    Board.setPWMServoPulse(6, 1500, 500)

def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)

count = 0
_stop = False
shape_list = []
get_roi = False
__isRunning = False
detect_shape = 'unidentified'
start_pick_up = False
start_count_t1 = True

# Variable reset
def reset():
    global _stop
    global count
    global get_roi
    global shape_list
    global detect_shape
    global start_pick_up
    global __target_shape
    global start_count_t1

    count = 0
    _stop = False
    shape_list = []
    get_roi = False
    __target_shape = ()
    detect_shape = 'unidentified'
    start_pick_up = False
    start_count_t1 = True

# App initialization call
def init():
    print("ShapeSorting Init")
    # The light is turned off by default after the ultrasonic wave is turned on
    HWSONAR.setRGBMode(0)
    HWSONAR.setPixelColor(0, Board.PixelColor(0,0,0))
    HWSONAR.setPixelColor(1, Board.PixelColor(0,0,0))    
    HWSONAR.show()
    initMove()

# App starts playing method call
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ShapeSorting Start")

# App stops playing method calls
def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ShapeSorting Stop")

# App exit gameplay call
def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ShapeSorting Exit")

rect = None
size = (640, 480)
rotation_angle = 0
unreachable = False 
world_X, world_Y = 0, 0
def move():
    global rect
    global _stop
    global get_roi
    global unreachable
    global __isRunning
    global detect_shape
    global start_pick_up
    global rotation_angle
    global world_X, world_Y
    
    # Place coordinates
    coordinate = {
        'circle':   (18, 9, 2),
        'triangle': (-18, 9,  2),
        'square':  (-18, 0, 2),
        'capture': (0, 16.5, 2),
        'pick': (0, 19, 0)
    }
    
    while True:
        if __isRunning:        
            if detect_shape != 'unidentified' and start_pick_up:  # If a shape is detected, start clamping
                
                setBuzzer(0.1)     # Set the buzzer to sound for 0.1 seconds
                
                if not __isRunning:  # Check whether to stop playing
                    continue
                Board.setPWMServoPulse(1, 2000, 500) # Open claws
                time.sleep(1.5)
                if not __isRunning:
                    continue
                    
                result = AK.setPitchRangeMoving((coordinate['pick'][0], coordinate['pick'][1], coordinate['pick'][2]), -90, -90, 90) # Run to above the coordinates
                
                if result == False:
                    unreachable = True
                    print("Unreachable\n")
                else:
                    unreachable = False
                    time.sleep(result[2] / 1000) #If the specified location can be reached, get the running time
                    
                    # 2nd trial
                if unreachable:
                    result = AK.setPitchRangeMoving((coordinate['pick'][0], coordinate['pick'][1], coordinate['pick'][2]), -90, -90, 90) # Run to above the coordinates

                Board.setPWMServoPulse(1, 1200, 500) # Close paw
                time.sleep(1.5)

                    # Motion in between picks, elevate arm
                AK.setPitchRangeMoving((0, 6, 18), -90, -90, 90, 1500)
                time.sleep(1.5)
                

                if not __isRunning:
                    continue
                if detect_shape == 'circle':       # According to the detected shape, the robot arm rotates to the corresponding angle
                    
                    result = AK.setPitchRangeMoving((coordinate['circle'][0], coordinate['circle'][1], coordinate['circle'][2]), -180, -90, 180) 
                    if result == False:
                        unreachable = True
                        print("Unreachable\n")
                    else:
                        unreachable = False
                        time.sleep(result[2] / 1000) #If the specified location can be reached, get the running time
                    # 2nd trial
                    if unreachable:
                        result = AK.setPitchRangeMoving((coordinate['circle'][0], coordinate['circle'][1], coordinate['circle'][2]), -180, -90, 180)
                    time.sleep(1)

                elif detect_shape == 'triangle':
                    
                    result = AK.setPitchRangeMoving((coordinate['triangle'][0], coordinate['triangle'][1], coordinate['triangle'][2]), -180, -90, 180) 
                    if result == False:
                        unreachable = True
                        print("Unreachable\n")
                    else:
                        unreachable = False
                        time.sleep(result[2] / 1000) #If the specified location can be reached, get the running time
                    # 2nd trial
                    if unreachable:
                        result = AK.setPitchRangeMoving((coordinate['triangle'][0], coordinate['triangle'][1], coordinate['triangle'][2]), -180, -90, 180)
                    time.sleep(1)

                elif detect_shape == 'square':

                    result = AK.setPitchRangeMoving((coordinate['square'][0], coordinate['square'][1], coordinate['square'][2]), -180, -90, 180) 
                    if result == False:
                        unreachable = True
                        print("Unreachable\n")
                    else:
                        unreachable = False
                        time.sleep(result[2] / 1000) #If the specified location can be reached, get the running time
                    # 2nd trial
                    if unreachable:
                        result = AK.setPitchRangeMoving((coordinate['square'][0], coordinate['square'][1], coordinate['square'][2]), -180, -90, 180)
                    
                    time.sleep(1)

                detect_shape = 'unidentified'
                get_roi = False
                start_pick_up = False
                initMove()              
        else:
            time.sleep(0.01)
          
# Run child thread
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()    

t1 = 0
roi = ()
center_list = []
last_x, last_y = 0, 0
length = 50
w_start = 200
h_start = 200
def run(img):
    global roi
    global rect
    global count
    global get_roi
    global center_list
    global unreachable
    global __isRunning
    global start_pick_up
    global rotation_angle
    global last_x, last_y
    global world_X, world_Y
    global start_count_t1, t1
    global detect_shape, shape_list
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]   

    if not __isRunning: # Check whether the gameplay is turned on, if not, return to the original image
        return img
    
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)     
    frame_gray = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2GRAY)  # Convert the image to grayscale
    ret, frame_thresh = cv2.threshold(frame_gray, 120, 255, cv2.THRESH_BINARY)

    contours, hierarchy = cv2.findContours(frame_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.04*cv2.arcLength(contour, True), True)
        cv2.drawContours(img, [approx], 0, (0, 0, 0), 5)

        M = cv2.moments(contour) 
        if M['m00'] != 0.0: 
            x = int(M['m10']/M['m00']) 
            y = int(M['m01']/M['m00'])

        shape = "unidentified"

        if len(approx) == 3:
            shape = "triangle"
        elif len(approx) == 4:
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = w / float(h)
            shape = "square" if aspect_ratio >= 0.95 and aspect_ratio <= 1.05 else "rectangle"
        elif len(approx) > 4:
            shape = "circle"

        if shape in __target_shape:
            detect_shape = shape
            start_pick_up = True
            cv2.drawContours(img, [approx], 0, (0, 255, 0), 5)

    cv2.putText(img, "Shape: " + detect_shape, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
    print("detected shape:\n")
    print(detect_shape)
    return img

if __name__ == '__main__':
    init()
    start()
    __target_shape = ('circle', 'triangle', 'square')
    cap = cv2.VideoCapture(-1)
    while True:
        ret, img = cap.read()
        if ret:
            frame = img.copy()
            Frame = run(frame)  
            frame_resize = cv2.resize(Frame, (320, 240))
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    cv2.destroyAllWindows()
