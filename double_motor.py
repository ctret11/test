from adafruit_servokit import ServoKit
import math
import numpy as np
import adafruit_bno055
import serial
import calcpoint
import board 
import busio
import time
import atexit



i2c_bus0 = (busio.I2C(board.SCL_1, board.SDA_1))
print("A")
kit = ServoKit(channels=16, i2c=i2c_bus0)
print("B")
kit.servo[0].set_pulse_width_range(1100, 1900)
kit.servo[1].set_pulse_width_range(1100, 1900)
print("C")
time.sleep(1)


def Motor():
    print("Motor init")
    kit.servo[0].angle = 100 #right 180 front 
    kit.servo[1].angle = 100 #left 0 front
    print("d")
    time.sleep(1)
    kit.servo[0].angle = 20
    kit.servo[1].angle = 20
    time.sleep(1.6)
    kit.servo[0].angle = 180
    kit.servo[1].angle = 20
    time.sleep(5)
    kit.servo[0].angle = 180
    kit.servo[1].angle = 20
    time.sleep(0.2)
    kit.servo[0].angle = 20
    kit.servo[1].angle = 180
    time.sleep(0.4)
    kit.servo[0].angle = 100
    kit.servo[1].angle = 100

def handle_exit():
    time.sleep(1)
    kit.servo[0].angle = 100
    kit.servo[1].angle = 100
atexit.register(handle_exit)

if __name__ == "__main__":
    Motor()








