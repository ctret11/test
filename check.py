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
i2c_bus0 = (busio.I2C(board.SCL_1, board.SDA_1))
sensor = adafruit_bno055.BNO055_I2C(i2c)
#sensor = adafruit_bno055.BNO055_I2C(i2c_bus0)
print("connect jetson")
kit = ServoKit(channels=16, i2c=i2c_bus0)
print("connect i2c")
kit.servo[0].set_pulse_width_range(1100, 1900) # servo 0 - servo motor
kit.servo[1].set_pulse_width_range(1100, 1900) # servo 1 - bldc motor
kit.servo[0].angle = 100 # default
kit.servo[1].angle = 100 # default
print("first default")
time.sleep(1)

def handle_exit():
    time.sleep(1)
    kit.servo[0].angle = 100
    kit.servo[1].angle = 100
atexit.register(handle_exit)

def get_yaw():  #check heading
    yaw = ((math.atan2(sensor.magnetic[1], sensor.magnetic[0]) * 180/math.pi)) % 360
    yaw -= 4
    return yaw

def Optimal(): #check optimal angle (lat-long/yaw minus)
    optimal = 0
    yaw = get_yaw()
    bearing = 0
    print("yaw: %f" %(yaw))
    optimal = bearing- yaw
    print("final_optiamls: %f" %(optimal))
    if abs(optimal)>180:
        if bearing > yaw:
            optimal = optimal - 360 # angle = -180 down => left 
            print("bozung_optimals: %f" %(optimal)) 
        elif bearing < yaw:
            optimal = optimal + 360 # angle = 180 up => right
            print("bozung_optimals: %f" %(optimal))

    Servo(optimal,bearing)

def Servo(angle,bearing): # angle to move (4.84~5 degree)
    newring = bearing
    print('kozung_bearing: %f' %(newring))
    if angle > 0:
        kit.servo[0].angle = 100
        kit.servo[1].angle = 20
        if 0<abs(angle)<=3:
            kit.servo[0].angle = 160
            kit.servo[1].angle = 40
            locationForRange(angle)
        elif 3<abs(angle)<=6:
            time.sleep(0.08)
        elif 6<abs(angle)<=12:
            time.sleep(0.15)
        elif 12<abs(angle)<=18:
            time.sleep(0.21)
        elif 18<abs(angle)<=20:
            time.sleep(0.26)
        elif 20<abs(angle)<=26:
            time.sleep(0.31)
        elif 26<abs(angle)<=32:
            time.sleep(0.37)
        elif 32<abs(angle)<=38:
            time.sleep(0.42)
        elif 38<abs(angle)<=40:
            time.sleep(0.45)
        elif 40<abs(angle)<=46:
            time.sleep(0.52)
        elif 46<abs(angle)<=52:
            time.sleep(0.58)
        elif 52<abs(angle)<=56:
            time.sleep(0.63)
        elif 56<abs(angle)<=60:
            time.sleep(0.68)
        elif 60<abs(angle)<=66:
            time.sleep(0.73)
        elif 66<abs(angle)<=70:
            time.sleep(0.78)
        elif 70<abs(angle)<=76:
            time.sleep(0.83)
        elif 76<abs(angle)<=82:
            time.sleep(0.88)
        elif 82<abs(angle)<=84:
            time.sleep(0.92)
        elif 84<abs(angle)<=90:
            time.sleep(0.98)
        elif abs(angle)>90:
            print("back turn") 
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(1.715)
            kit.servo[0].angle = 180
            kit.servo[1].angle = 180
            time.sleep(0.48)
            kit.servo[0].angle = 100
            kit.servo[1].angle = 100
            time.sleep(0.1) #check test
            newyaw = get_yaw()
            newtimal = newring-newyaw
            if abs(newtimal)>180:
                if newring > newyaw:
                    newtimal = newtimal - 360 # angle = -180 down => left  
                elif newring < newyaw:
                    newtimal = newtimal + 360 # angle = 180 up => right
            Servo(newtimal,newring)
            
    elif angle < 0:
        kit.servo[0].angle = 180
        kit.servo[1].angle = 100
        if 0<abs(angle)<=3:
            kit.servo[0].angle = 160
            kit.servo[1].angle = 40
            locationForRange(angle)
        elif 3<abs(angle)<=6:
            time.sleep(0.1)
        elif 6<abs(angle)<=12:
            time.sleep(0.16)
        elif 12<abs(angle)<=18:
            time.sleep(0.21)
        elif 18<abs(angle)<=20:
            time.sleep(0.26)
        elif 20<abs(angle)<=26:
            time.sleep(0.31)
        elif 26<abs(angle)<=32:
            time.sleep(0.37)
        elif 32<abs(angle)<=38:
            time.sleep(0.42)
        elif 38<abs(angle)<=40:
            time.sleep(0.45)
        elif 40<abs(angle)<=46:
            time.sleep(0.52)
        elif 46<abs(angle)<=52:
            time.sleep(0.58)
        elif 52<abs(angle)<=56:
            time.sleep(0.63)
        elif 56<abs(angle)<=60:
            time.sleep(0.68)
        elif 60<abs(angle)<=66:
            time.sleep(0.73)
        elif 66<abs(angle)<=70:
            time.sleep(0.78)
        elif 70<abs(angle)<=76:
            time.sleep(0.83)
        elif 76<abs(angle)<=82:
            time.sleep(0.88)
        elif 82<abs(angle)<=84:
            time.sleep(0.92)
        elif 84<abs(angle)<=90:
            time.sleep(0.98)
        elif abs(angle) > 90:
            print("back turn")
            kit.servo[0].angle = 180
            kit.servo[1].angle = 180
            time.sleep(1.715)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(0.48)
            kit.servo[0].angle = 100
            kit.servo[1].angle = 100
            time.sleep(1)  # check test
            newyaw = get_yaw()
            newtimal = newring-newyaw
            if abs(newtimal)>180:
                if newring > newyaw:
                    newtimal = newtimal - 360 # angle = -180 down => left  
                elif newring < newyaw:
                    newtimal = newtimal + 360 # angle = 180 up => right
            Servo(newtimal,newring)
    kit.servo[0].angle = 160
    kit.servo[1].angle = 40
    time.sleep(3)
    kit.servo[0].angle = 100
    kit.servo[1].angle = 100


if __name__ == '__main__':
    print("##################")
    while(True):
        Optimal()
