#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/mse112-ws/MasterPi/')
import cv2
import time
import Camera
import threading
import logging
import yaml_handle
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Sonar as Sonar
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *


if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()
HWSONAR = Sonar.Sonar() #ultrasonic sensor

range_rgb = {
    'red':   (0, 0, 255),
    'blue':  (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

lab_data = None
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

    __target_color = ('red')
# Set detection color
def setTargetColor(target_color):
    global __target_color

    print("COLOR", target_color)
    __target_color = target_color
    return (True, ())

#Find the contour with the largest area
#The parameter is a list of contours to be compared
def getAreaMaxContour(contours) :
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours : #Traverse all contours
            contour_area_temp = math.fabs(cv2.contourArea(c))  #Calculate contour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  #Only when the area is greater than 300, the outline of the largest area is effective to filter interference
                    area_max_contour = c

        return area_max_contour, contour_area_max  #Return the largest contour

# The angle at which the gripper closes when clamping
servo1 = 1500

# initial position
def initMove():
    Board.setPWMServoPulse(1, servo1, 800)
    AK.setPitchRangeMoving((0, 8, 10), -90, -90, 0, 1500)

def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)

# #Set the RGB light color of the expansion board to match the color to be tracked
# def set_rgb(color):
#     if color == "red":
#         Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
#         Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
#         Board.RGB.show()
#     elif color == "green":
#         Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
#         Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
#         Board.RGB.show()
#     elif color == "blue":
#         Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
#         Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
#         Board.RGB.show()
#     else:
#         Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
#         Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
#         Board.RGB.show()

count = 0
_stop = False
color_list = []
get_roi = False
__isRunning = False
detect_color = 'None'
start_pick_up = False
start_count_t1 = True

#Variable reset
def reset():
    global _stop
    global count
    global get_roi
    global color_list
    global detect_color
    global start_pick_up
    global __target_color
    global start_count_t1

    count = 0
    _stop = False
    color_list = []
    get_roi = False
    __target_color = ()
    detect_color = 'None'
    start_pick_up = False
    start_count_t1 = True

# app initialization call
def init():
    print("ColorSorting Init")
    # # After ultrasonic is turned on, the light is turned off by default
    # HWSONAR.setRGBMode(0)
    # HWSONAR.setPixelColor(0, Board.PixelColor(0,0,0))
    # HWSONAR.setPixelColor(1, Board.PixelColor(0,0,0))    
    # HWSONAR.show()
    load_config()
    initMove()

# app starts gameplay call
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorSorting Start")

# app stops gameplay calling
def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    # set_rgb('None')
    print("ColorSorting Stop")

# app exit gameplay call
def exit():
    global _stop
    global __isRunning
    _stop = True
    # set_rgb('None')
    __isRunning = False
    print("ColorSorting Exit")


def move():
    global _stop
    global get_roi
    global unreachable
    global __isRunning
    
    #coordinates for pick and place
    coordinate = {
        'place':   (18, 0, 4),
        'pick': (-18, 9,  2),
    }

    while True:
        if __isRunning:
                
                # initMove()
                
                # Pick
                print("Pick and Place Start\n")
                if not __isRunning:
                    continue

                # Pick
                print("Pick and Place Start\n")

                Board.setPWMServoPulse(1, 2000, 500) # open claws
                time.sleep(2.5)

                result = AK.setPitchRangeMoving((coordinate['pick'][0], coordinate['pick'][1], coordinate['pick'][2]), -90, -90, 0) # Run to above the coordinates of the corresponding color
                if result == False:
                    unreachable = True
                    print("Unreachable\n")
                else:
                    unreachable = False
                    time.sleep(result[2]/1000) #If the specified location can be reached, get the running time

                if not __isRunning:
                    continue

                AK.setPitchRangeMoving((coordinate['pick']), -90, -90, 0, 500)  # pick from the corresponding coordinate
                time.sleep(0.5)

                if not __isRunning:
                    continue


                Board.setPWMServoPulse(1, 1500, 500) # closed paw
                time.sleep(1.5)

                if not __isRunning:
                    continue

                # end of Pick


                AK.setPitchRangeMoving((0, 6, 18), 0,-90, 90, 1500)
                time.sleep(1.5)

                
                # Place

                result = AK.setPitchRangeMoving((coordinate['place'][0], coordinate['place'][1], coordinate['place'][2]), -90, -90, 0) # Run to above the coordinates of the corresponding color
                if result == False:
                    unreachable = True
                    print("Unreachable\n")
                else:
                    unreachable = False
                    time.sleep(result[2]/1000) #If the specified location can be reached, get the running time

                if not __isRunning:
                    continue

                AK.setPitchRangeMoving((coordinate['place']), -90, -90, 0, 500)  # pick from the corresponding coordinate
                time.sleep(1.5)

                if not __isRunning:
                    continue

                Board.setPWMServoPulse(1, 1800, 500) # open claws
                time.sleep(1.5)

                if not __isRunning:
                    continue

                # end of Place


                Board.setPWMServosPulse([1200, 4, 1,1500, 3,515, 4,2170, 5,945]) # Robotic arm resets
                time.sleep(1.2)

                __isRunning = False

                print("Pick and Place end\n")

                initMove()

                if not __isRunning:
                    continue

                


#Run child thread
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()


if __name__ == '__main__':
    init()
    start()

    th.join()

    # while True:
    #     print("in while loop\n")
    # __target_color = ('red', 'green', 'blue')
    # cap = cv2.VideoCapture('http://127.0.0.1:8080?action=stream')
    # while True:
    #     ret,img = cap.read()
    #     if ret:
    #         frame = img.copy()
    #         Frame = run(frame)  
    #         frame_resize = cv2.resize(Frame, (320, 240))
    #         cv2.imshow('frame', frame_resize)
    #         key = cv2.waitKey(1)
    #         if key == 27:
    #             break
    #     else:
    #         time.sleep(0.01)
    # cv2.destroyAllWindows()