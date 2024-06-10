#!/usr/bin/python3
#coding=utf8
import sys
sys.path.append('/home/pi/mse112-ws/MasterPi/')
import time
import signal
import numpy as np
import pandas as pd
import HiwonderSDK.Sonar as Sonar
import HiwonderSDK.Board as Board



HWSONAR = Sonar.Sonar()

while True:
    dist = HWSONAR.getDistance() / 10.0

    print("Distance measured:\n")
    print(dist)

