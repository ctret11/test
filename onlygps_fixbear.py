
#!/usr/bin/env python
# -*- coding: utf-8 -*-
from statistics import mean
import serial
import operator
import collections
import calcpoint
import math
from functools import reduce
import numpy as np
from adafruit_servokit import ServoKit
import adafruit_bno055
import board
import busio
import time
import atexit

i2c = board.I2C()    
sensor = adafruit_bno055.BNO055_I2C(i2c)
print("Quaternion: {}" .format(sensor.quaternion))
print("mag: {}" .format(sensor.magnetic))

def get_yaw():
    yaw = ((math.atan2(sensor.magnetic[1], sensor.magnetic[0]) * 180/math.pi)) % 360
    return yaw

def optimal():
    lat1 = 35.2321266
    lon1 = 129.0793002


    lat2 = 35.232088
    lon2 = 129.079293

    φ1 = lat1 * math.pi / 180
    φ2 = lat2 * math.pi / 180
    λ1 = lon1 * math.pi / 180
    λ2 = lon2 * math.pi / 180

    y = math.sin(λ2 - λ1) * math.cos(φ2)
    x = math.cos(φ1) * math.sin(φ2) - math.sin(φ1) * math.cos(φ2) * math.cos(λ2 - λ1)
    θ = math.atan2(y, x)
    bearing = (θ * 180 / math.pi + 360) % 360
    yaw = get_yaw()
    optimal = bearing- yaw # bearing > yaw => original + => + right
    if abs(optimal)> 180: # change charge low 
        if bearing > yaw: # but bearing low than yaw for eular # yaw + 2pi ex bear = 300 yaw = 10(370)
            optimal = optimal - 360 # angle = -180 down => left  
        elif bearing < yaw: # bearing + 2pi bear = 10(370) yaw = 300
            optimal = optimal + 360 # angle = 180 up => right
    
    print('bearing: %f' %(bearing))
    print('yaw: %f' %(yaw))
    print('optimal: %f' %(optimal))



# rl1 = lat1 * math.pi / 180
# rl2 = lat2 * math.pi / 180
# mrl = (lat2 - lat1) * math.pi / 180
# mrlo = (lon2 - lon1) * math.pi / 180
# alo = math.sin(mrl/2)*math.sin(mrl/2)+ math.cos(rl1)*math.cos(rl2)*math.sin(mrlo/2)*math.sin(mrlo/2)
# clo = 2*math.atan2(math.sqrt(alo),math.sqrt(1-alo))
# range1 = 6371e3 * clo #meter range point 0 - point 1
# print('bearing: %f' %(range1))

if __name__ == '__main__':
    print("##################")
    while True:
        optimal()
