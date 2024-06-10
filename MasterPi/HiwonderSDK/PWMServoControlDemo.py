#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/mse112-ws/MasterPi/')
import time
import signal
import threading
import HiwonderSDK.Board as Board

start = True
#Before closing
def Stop(signum, frame):
    global start

    start = False
    print('Closing...')

signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    
    while True:
        Board.setPWMServoPulse(1, 1500, 1000) # Set the pulse width of Servo 1 to 1500 and the running time to 1000 milliseconds
        time.sleep(1)
        Board.setPWMServoPulse(1, 2500, 1000) # Set the pulse width of Servo 1 to 2500 and the running time to 1000 milliseconds
        time.sleep(1)
        
        if not start:
            Board.setPWMServoPulse(1, 1500, 1000) # Set the pulse width of Servo 1 to 1500 and the running time to 1000 milliseconds
            time.sleep(1)
            print('closed')
            break
    
    
        