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
#import adafruit_bno055

ser = serial.Serial(port = "/dev/ttyACM0", baudrate = 57600, timeout = 0.1)
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
#print("Quaternion: {}" .format(sensor.quaternion))

global hopping_count
hopping_count = 0
global points
points = [[35.069481,128.578891],[35.069618,128.578871],[35.069464,128.578968],[35.069558,128.578930],[35.069368,128.578999]] # 369 /984

def handle_exit():
    time.sleep(1)
    kit.servo[0].angle = 100
    kit.servo[1].angle = 100
atexit.register(handle_exit)
        

def GPSparser(data):
	gps_data = data.split(b",")
	idx_rmc = data.find(b'GNGGA')
	if data[idx_rmc:idx_rmc+5] == b"GNGGA":
		data = data[idx_rmc:]    
		if checksum(data):
		   # print(data)
			parsed_data = data.split(b",")
			print(parsed_data)
			return parsed_data
		else :
			print ("checksum error")

def checksum(sentence):
	sentence = sentence.strip(b'\n')
	nmeadata, cksum = sentence.split(b'*',1)
	calc_cksum = reduce(operator.xor, (ord(chr(s)) for s in nmeadata), 0)
	print(int(cksum,16), calc_cksum)
	if int(cksum,16) == calc_cksum:
		return True 
	else:
		return False 

def get_yaw():  #check heading
    yaw = ((math.atan2(sensor.magnetic[1], sensor.magnetic[0]) * 180/math.pi)) % 360
    
    return yaw

def Optimal(lat_1,lon_1,lat_2,lon_2): #check optimal angle (lat-long/yaw minus)
    global hopping_count   #delete
    global points #delete
    optimal = 0
    φ1 = lat_1 * math.pi / 180
    φ2 = lat_2 * math.pi / 180
    λ1 = lon_1 * math.pi / 180
    λ2 = lon_2 * math.pi / 180
    y = math.sin(λ2 - λ1) * math.cos(φ2)
    x = math.cos(φ1) * math.sin(φ2) - math.sin(φ1) * math.cos(φ2) * math.cos(λ2 - λ1)
    θ = math.atan2(y, x)
    bearing = ((θ * 180 / math.pi + 360)) % 360
    print("optimal_First: %f" %(bearing))
    yaw = get_yaw()
    print("yaw: %f" %(yaw))
    if hopping_count == 0: #delete
       print("calibrate time") #delete
       time.sleep(7) #delete
    optimal = bearing- yaw
    print("final_optiamls: %f" %(optimal))
    if abs(optimal)>180:
        if bearing > yaw:
            optimal = optimal - 360 # angle = -180 down => left  
        elif bearing < yaw:
            optimal = abs(abs(optimal) - 360) # angle = 180 up => right

    return optimal,bearing

#def location(): # receive gps example
#	
#    data = ser.readline()
#    result = collections.defaultdict()
#    res = GPSparser(data)
#    if res == None:
#        return(0,0)
#    else:
#        print(res)
#        lat = str (res[2])
#        lon = str (res[4])
#        if (res == "checksum error"):
#            print("")
#            print(lat)
#        else:
#            lat_h = float(lat[2:4])
#            lon_h = float(lon[2:5])
#            lat_m = float(lat[4:12])
#            lon_m = float(lon[5:13])
#            print('lat_h: %f lon_h: %f lat_m: %f lon_m: %f' %(lat_h, lon_h, lat_m, lon_m))
#            latitude = lat_h + (lat_m/60)
#            longitude = lon_h + (lon_m/60)
#            print('latitude: %f longitude: %f' %(latitude,longitude))
#
#    return latitude,longitude
#    # Optimal(latitude,longitude,first_lon)


def locationForAngle(): #receive gps to change variable number
    global hopping_count
    global points
    if hopping_count == 5:
        return
    sum = 0
    cnt = 1
    while True:
        data = ser.readline()
        result = collections.defaultdict()
        res = GPSparser(data)
        if res == None:
            continue
        else:
            print(res)
            lat = str (res[2])
            lon = str (res[4])
            if (res == "checksum error"):
                print("")
                print(lat)
            else:
                if cnt%3 == 0:
                    Servo(second,bearing)
                lat_h = float(lat[2:4])
                lon_h = float(lon[2:5])
                lat_m = float(lat[4:12])
                lon_m = float(lon[5:13])
                print('lat_h: %f lon_h: %f lat_m: %f lon_m: %f' %(lat_h, lon_h, lat_m, lon_m))
                latitude = lat_h + (lat_m/60)
                longitude = lon_h + (lon_m/60)
                print('latitude: %f longitude: %f' %(latitude,longitude))
                second,bearing = Optimal(latitude,longitude,points[hopping_count][0],points[hopping_count][1])
                cnt += 1

def Servo(angle,bearing): # angle to move (4.84~5 degree)
    newring = bearing
    print('anglereal: %f' %(angle))
    if angle > 0:
        kit.servo[0].angle = 20
        kit.servo[1].angle = 20
        if 0<abs(angle)<=5:
            kit.servo[0].angle = 170
            kit.servo[1].angle = 30
            locationForRange(angle)
        elif 5<abs(angle)<=10:
            time.sleep(0.325)
        elif 10<abs(angle)<=15:
            time.sleep(0.35)
            kit.servo[0].angle = 180
            kit.servo[1].angle = 180
            time.sleep(0.05)     
        elif 15<abs(angle)<=20:
            time.sleep(0.4)
            kit.servo[0].angle = 180
            kit.servo[1].angle = 180
            time.sleep(0.05)  
        elif 20<abs(angle)<=25:
            time.sleep(0.45)
            kit.servo[0].angle = 180
            kit.servo[1].angle = 180
            time.sleep(0.05)    
        elif 25<abs(angle)<=30:
            time.sleep(0.47)
            kit.servo[0].angle = 180
            kit.servo[1].angle = 180
            time.sleep(0.0)
        elif 30<abs(angle)<=35:
            time.sleep(0.5)
            kit.servo[0].angle = 180
            kit.servo[1].angle = 180
            time.sleep(0.05)
        elif 35<abs(angle)<=40:
            time.sleep(0.525)
            kit.servo[0].angle = 180
            kit.servo[1].angle = 180
            time.sleep(0.075)
        elif 40<abs(angle)<=45:
            time.sleep(0.55)
            kit.servo[0].angle = 180
            kit.servo[1].angle = 180
            time.sleep(0.05)
        elif 45<abs(angle)<=50:
            time.sleep(0.57)
            kit.servo[0].angle = 180
            kit.servo[1].angle = 180
            time.sleep(0.07)
        elif 50<abs(angle)<=55:
            time.sleep(0.6)
            kit.servo[0].angle = 180
            kit.servo[1].angle = 180
            time.sleep(0.1)
        elif 55<abs(angle)<=60:
            time.sleep(0.62)
            kit.servo[0].angle = 180
            kit.servo[1].angle = 180
            time.sleep(0.12)
        elif 60<abs(angle)<=65:
            time.sleep(0.65)
            kit.servo[0].angle = 180
            kit.servo[1].angle = 180
            time.sleep(0.15)
        elif 65<abs(angle)<=70:
            time.sleep(0.67)
            kit.servo[0].angle = 180
            kit.servo[1].angle = 180
            time.sleep(0.17)
        elif 70<abs(angle)<=75:
            time.sleep(0.7)
            kit.servo[0].angle = 180
            kit.servo[1].angle = 180
            time.sleep(0.2)
        elif 75<abs(angle)<=80:
            time.sleep(0.72)
            kit.servo[0].angle = 180
            kit.servo[1].angle = 180
            time.sleep(0.22)
        elif 80<abs(angle)<=85:
            time.sleep(0.75)
            kit.servo[0].angle = 180
            kit.servo[1].angle = 180
            time.sleep(0.25)
        elif 85<abs(angle)<=90:
            time.sleep(0.8)
            kit.servo[0].angle = 180
            kit.servo[1].angle = 180
            time.sleep(0.3)
        elif abs(angle) > 90:
            print("back turn")
            kit.servo[0].angle = 180
            kit.servo[1].angle = 180
            time.sleep(1.6)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(0.4)
            kit.servo[0].angle = 180
            kit.servo[1].angle = 20
            time.sleep(0.2)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 180
            time.sleep(0.4)
            kit.servo[0].angle = 100
            kit.servo[1].angle = 100  # check test
            newyaw = get_yaw()
            newtimal = newring-newyaw
            if abs(newtimal)>180:
                if newring > newyaw:
                    newtimal = newtimal - 360 # angle = -180 down => left  
                elif newring < newyaw:
                    newtimal = abs(abs(newtimal) - 360) # angle = 180 up => right
            Servo(newtimal,newring)
            
    elif angle < 0:
        kit.servo[0].angle = 180
        kit.servo[1].angle = 180
        if 0<abs(angle)<=5:
            kit.servo[0].angle = 170
            kit.servo[1].angle = 30
            locationForRange(angle)
        elif 5<abs(angle)<=10:
            time.sleep(0.325)
        elif 10<abs(angle)<=15:
            time.sleep(0.35)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(0.05)     
        elif 15<abs(angle)<=20:
            time.sleep(0.4)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(0.05)  
        elif 20<abs(angle)<=25:
            time.sleep(0.45)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(0.05)    
        elif 25<abs(angle)<=30:
            time.sleep(0.47)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(0.0)
        elif 30<abs(angle)<=35:
            time.sleep(0.5)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(0.05)
        elif 35<abs(angle)<=40:
            time.sleep(0.525)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(0.075)
        elif 40<abs(angle)<=45:
            time.sleep(0.55)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(0.05)
        elif 45<abs(angle)<=50:
            time.sleep(0.57)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(0.07)
        elif 50<abs(angle)<=55:
            time.sleep(0.6)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(0.1)
        elif 55<abs(angle)<=60:
            time.sleep(0.62)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(0.12)
        elif 60<abs(angle)<=65:
            time.sleep(0.65)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(0.15)
        elif 65<abs(angle)<=70:
            time.sleep(0.67)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(0.17)
        elif 70<abs(angle)<=75:
            time.sleep(0.7)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(0.2)
        elif 75<abs(angle)<=80:
            time.sleep(0.72)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(0.22)
        elif 80<abs(angle)<=85:
            time.sleep(0.75)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(0.25)
        elif 85<abs(angle)<=90:
            time.sleep(0.8)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(0.3)
        elif abs(angle) > 90:
            print("back turn")
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(1.6)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 20
            time.sleep(0.4)
            kit.servo[0].angle = 180
            kit.servo[1].angle = 20
            time.sleep(0.2)
            kit.servo[0].angle = 20
            kit.servo[1].angle = 180
            time.sleep(0.4)
            kit.servo[0].angle = 100
            kit.servo[1].angle = 100  # check test
            newyaw = get_yaw()
            newtimal = newring-newyaw
            if abs(newtimal)>180:
                if newring > newyaw:
                    newtimal = newtimal - 360 # angle = -180 down => left  
                elif newring < newyaw:
                    newtimal = abs(abs(newtimal) - 360) # angle = 180 up => right
            Servo(newtimal,newring)
    kit.servo[0].angle = 170
    kit.servo[1].angle = 30
    locationForRange(angle)

def locationForRange(angle): #harversine range to stopping ship
    global hopping_count
    global points
    latitudes = points[hopping_count][0]
    longitudes = points[hopping_count][1]
    while True:
        data = ser.readline()
        result = collections.defaultdict()
        res = GPSparser(data)
        if res == None:
            continue
        else:
            print(res)
            lat = str (res[2])
            lon = str (res[4])
            if (res == "checksum error"):
                print("")
                print(lat)
            else:
                lat_h = float(lat[2:4])
                lon_h = float(lon[2:5])
                lat_m = float(lat[4:12])
                lon_m = float(lon[5:13])
                print('lat_h: %f lon_h: %f lat_m: %f lon_m: %f' %(lat_h, lon_h, lat_m, lon_m))
                latitude = lat_h + (lat_m/60)
                longitude = lon_h + (lon_m/60)
                print('latitude_next: %f longitude_next: %f' %(latitude,longitude))
                rl1 = latitude * math.pi / 180
                rl2 = latitudes * math.pi / 180
                mrl = (latitudes - latitude) * math.pi / 180
                mrlo = (longitudes - longitude) * math.pi / 180
                alo = math.sin(mrl/2)*math.sin(mrl/2)+ math.cos(rl1)*math.cos(rl2)*math.sin(mrlo/2)*math.sin(mrlo/2)
                clo = 2*math.atan2(math.sqrt(alo),math.sqrt(1-alo))
                range = 6371e3 * clo #meter range point 0 - point 1
                #range = 6 * math.sqrt((latitude-latitudes)*(latitude-latitudes)+(longitude-longitudes)*(longitude-longitudes)) / math.sqrt(0.000049*0.000049+0.000018*0.000018)
                print('range_:{0}'.format(range))
                if (range < 2.4):  # stop front start behind wheel 
                    hopping_count += 1
                    kit.servo[0].angle = 20
                    kit.servo[1].angle = 180
                    time.sleep(0.5)
                    kit.servo[0].angle = 100
                    kit.servo[1].angle = 100
                    locationForAngle()

        
if __name__ == '__main__':
    print("##################")
    while True:
        locationForAngle()
