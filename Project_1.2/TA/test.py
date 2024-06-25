#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/mse112-ws/MasterPi//')
import cv2
import time
import math
import signal
import Camera
import threading
import numpy as np
import yaml_handle
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Sonar as Sonar
import HiwonderSDK.Misc as Misc
import HiwonderSDK.Board as Board
from HiwonderSDK.PID import PID
import pandas as pd

# initialization
AK = ArmIK()
pitch_pid = PID(P=0.28, I=0.16, D=0.18)

HWSONAR = Sonar.Sonar()
distance = 0

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}
img_centerx = 320
# Variable for distance obstacle avoidance
distance_data = []
stopMotor = False
Threshold = 15  # Set threshold for obstacle distance

# line tracking
roi = [ # [ROI, weight]
        (240, 280,  0, 640, 0.1), 
        (340, 380,  0, 640, 0.3), 
        (430, 460,  0, 640, 0.6)
       ]

roi_h1 = roi[0][0]
roi_h2 = roi[1][0] - roi[0][0]
roi_h3 = roi[2][0] - roi[1][0]

roi_h_list = [roi_h1, roi_h2, roi_h3]
size = (640, 480)

# Line patrol
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

def servo_init():
    Board.setPWMServoPulse(1, 2500, 300) # Set the pulse width of Servo 1 to 2500 and the running time to 1000 milliseconds
    time.sleep(1)
    Board.setPWMServoPulse(3, 1000, 300) 
    time.sleep(1)
    Board.setPWMServoPulse(4, 2000, 1000) 
    time.sleep(1)
    Board.setPWMServoPulse(5, 2100, 1000) 
    time.sleep(1)
    Board.setPWMServoPulse(6, 1500, 1000) 

# Set the detection color
def setTargetColor(target_color):
    global __target_color

    print("COLOR", target_color)
    __target_color = target_color
    return (True, ())

lab_data = None

def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

# initial position
def initMove():
    servo_init()
    MotorStop()

line_centerx = -1
# Variable reset
def reset():
    global line_centerx
    global __target_color
    
    line_centerx = -1
    __target_color = ()
    
# app initialization call
def init():
    print("VisualPatrol Init")
    load_config()
    initMove()

__isRunning = False
# app starts playing method call
def start():
    reset()

    global __isRunning
    global stopMotor
    global forward
    global turn
    global obstacle
    obstacle = False
    turn = True
    forward = True
    stopMotor = True
    __isRunning = True

    print("Line tracker 1.1 Start")

# app stops playing method calls
def stop():
    global __isRunning
    __isRunning = False
    MotorStop()
    print("Line tracker 1.1 Stop")

# app exit gameplay call
def exit():
    global __isRunning
    __isRunning = False
    MotorStop()
    print("Line tracker 1.1 Exit")

def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)

def MotorStop():
    Board.setMotor(1, 0) 
    Board.setMotor(2, 0)
    Board.setMotor(3, 0)
    Board.setMotor(4, 0)

# Close before processing
def Stop(signum, frame):
    global __isRunning
    
    __isRunning = False
    print('Closing...')
    MotorStop()  # Turn off all motors

# Find the contour with the largest area
# The parameter is a list of contours to be compared
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # Iterate over all contours
        contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate the contour area
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp >= 5:  # Only when the area is greater than 300, the contour of the largest area is valid to filter out interference
                area_max_contour = c

    return area_max_contour, contour_area_max  # Return the largest contour

def handle_obstacle():
    global obstacle

    while True:
        if obstacle:
            time.sleep(0.01)
            # Pick
            print("Pick and Place Start\n")

            print("The obstacle distance is :\n")
            print(coordinate['pick'][1])
            print("--------------------------\n")

            Board.setPWMServoPulse(1, 2000, 500) # Open claws
            time.sleep(2.5)

            result = AK.setPitchRangeMoving((coordinate['pick'][0], coordinate['pick'][1], coordinate['pick'][2]), -90, -90, 90) # Run to above the coordinates of the corresponding color
            if result == False:
                unreachable = True
                print("Unreachable\n")
            else:
                unreachable = False
                time.sleep(result[2] / 1000) #If the specified location can be reached, get the running time

            # AK.setPitchRangeMoving((coordinate['pick']), -90, -90, 90, 500)  # Pick from the corresponding coordinate
            time.sleep(0.5)

            Board.setPWMServoPulse(1, 1500, 500) # Close paw
            time.sleep(1.5)

            # Motion in between picks, elevate arm
            AK.setPitchRangeMoving((0, 6, 18), -90, -90, 90, 1500)
            time.sleep(1.5)

            # Place
            result = AK.setPitchRangeMoving((coordinate['place'][0], coordinate['place'][1], coordinate['place'][2]), -90, -90, 0) # Run to above the coordinates of the corresponding color
            if result == False:
                unreachable = True
                print("Unreachable\n")
            else:
                unreachable = False
                time.sleep(result[2] / 1000) #If the specified location can be reached, get the running time

            # AK.setPitchRangeMoving((coordinate['place']), -90, -90, 90, 500)  # Pick from the corresponding coordinate
            time.sleep(1.5)

            Board.setPWMServoPulse(1, 1800, 1000) # Open claws
            time.sleep(1.5)

            initMove()

            time.sleep(1.5)
            # Board.setPWMServoPulse(6, 1500, 2000) 
            
            obstacle = False
            print("Pick and Place end\n")

            time.sleep(1.5)

            initMove()
        else:
            time.sleep(0.01)

def move():
    global line_centerx
    global obstacle

    i = 0
    while True:
        coordinate = {
            'place': (-18, 2, 2),
            'pick': (0, globals()['distance'] + 3.5, 2),  # Adjusting y-coordinate based on the distance
        }
        if __isRunning:
            if line_centerx != -1 and not obstacle:
                num = (line_centerx - img_centerx)
                if abs(num) <= 5:  # The deviation is small and no processing is performed
                    pitch_pid.SetPoint = num
                else:
                    pitch_pid.SetPoint = 0
                pitch_pid.update(num) 
                tmp = pitch_pid.output    # Get PID output value
                tmp = 100 if tmp > 100 else tmp   
                tmp = -100 if tmp < -100 else tmp
                base_speed = Misc.map(tmp, -100, 100, -40, 40)  # Speed ​​mapping
                Board.setMotor(1, int(40 - base_speed)) # Set motor speed
                Board.setMotor(2, int(40 + base_speed))
                Board.setMotor(3, int(40 - base_speed))
                Board.setMotor(4, int(40 + base_speed))
            else:
                Board.setMotor(1, 40)
                Board.setMotor(2, 40)
                Board.setMotor(3, 40)
                Board.setMotor(4, 40)
        else:
            time.sleep(0.01)
            MotorStop()

        globals()['distance'] = HWSONAR.getDistance()
        if globals()['distance'] <= Threshold:
            print(f"Distance: {globals()['distance']}")
            obstacle = True
            Board.setMotor(1, 0)
            Board.setMotor(2, 0)
            Board.setMotor(3, 0)
            Board.setMotor(4, 0)
        i += 1
        if i > 3:
            i = 0

def run():
    signal.signal(signal.SIGINT, Stop)
    init()
    start()
    obstacle_thread = threading.Thread(target=handle_obstacle)
    obstacle_thread.start()
    try:
        while True:
            move()
    except KeyboardInterrupt:
        stop()
        exit()

if __name__ == '__main__':
    run()
