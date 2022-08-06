#!/usr/bin/env python3

import time
import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Point32
from std_msgs.msg import String, UInt8
import numpy as np
import math
import cv2
import adafruit_pca9685

from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus, PointCloud, LaserScan
from mecha_autoship_interfaces.srv import Battery, Actuator, Color
from adafruit_servokit import ServoKit
import board
import busio

def map(x, input_min, input_max, output_min, output_max):
    res = (x - input_min) * (output_max - output_min) / (
        input_max - input_min
    ) + output_min
    return int(output_min if res < output_min else res)
 
  
i2c_bus0 = (busio.I2C(board.SCL_1, board.SDA_1))
kit = ServoKit(channels=16,i2c=i2c_bus0)     
pca = adafruit_pca9685.PCA9685(i2c_bus0)
pca.frequency = 75

# kit.servo[0].set_pulse_width_range(1100, 1900) # servo 0 - bldc motor
# kit.servo[1].set_pulse_width_range(1100, 1900) # servo 1 - bldc motor
time.sleep(1)
kit.servo[0].angle = 100 # 
kit.servo[1].angle = 100 #  
time.sleep(3)

class MechaAutoshipExampleNode(Node):
    def __init__(self):
        super().__init__("mecha_autoship_example_node")
        self.get_logger().info("mecha_autoship_example_node Start")
        # ser = serial.Serial(port = "/dev/ttyACM0", baudrate = 38400, timeout = 0.1)    
        
        
        self.data = {
            "IMU": Imu(),
            "GPS": NavSatFix(),
            "LiDAR": PointCloud(),
            "Shape": String()
	    # "LiDAR": LaserScan(),
        }

        # 센서 데이터 Subscribe
        self.imu_sub_handler = self.create_subscription(
            Imu, "imu/data", self.imu_sub_callback, 10
        )
        self.gps_sub_handler = self.create_subscription(
            NavSatFix, "gps/data", self.gps_sub_callback, 10
        )
        self.lidar_sub_handler = self.create_subscription(
            PointCloud, "scan_points", self.lidar_sub_callback, 10
        )
        self.shape_sub_handler = self.create_subscription(String, "Shape", self.shape_sub_callback, 10)
        # # 서비스 Client 생성
        self.actuator_set_handler = self.create_client(Actuator, 'set_actuator')
        self.color_set_handler = self.create_client(Color, "set_color")
        # self._publisher = self.create_publisher(UInt8,"Angle",1)

        # 특정 토픽으로부터 데이터를 가져오는 예시입니다. 연결되는 콜백 함수를 참고하세요.
        #self.print_imu_data_example = self.create_timer(
        #    5, self.print_imu_data_example_callback
        #)
        self.risk_calculator_lidar_data = self.create_timer(1, self.risk_calculator_lidar_data_callback)
        # self.light_timer = self.create_timer(1, self.light_callback)

        

        # # RGB LED의 색상을 빨간색으로 변경하는 예시입니다.
        # self.set_pixel(255, 0, 0)
        # time.sleep(3)
        # self.set_pixel(100, 100, 100)  # 기본 상태로 복귀
        
        # #모터의 쓰로틀을 100%, 각도를 100도로 설정하는 예시입니다.
        

    def GPSparser(data):
        gps_data = data.split(b",")
        idx_rmc = data.find(b'GNGGA')
        if data[idx_rmc:idx_rmc+5] == b"GNGGA":
            data = data[idx_rmc:]    
            if checksum(data):
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

    def Optimal(lat_1,lon_1,lat_2,lon_2):
        optimal = 199.798876356 - (math.atan((lon_2-lon_1)/(lat_2-lat_1)) * 180 / math.pi +180)
        print('cool: %f' %(optimal))
        return optimal

    def location():
        data = ser.readline()
        result = collections.defaultdict()
        res = GPSparser(data)
        if res == None:
            return(0,0)
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
                print('latitude: %f longitude: %f' %(latitude,longitude))
                
    def shape_sub_callback(self, data):
        self.data["Shape"] = data

    def imu_sub_callback(self, data):
        self.data["IMU"] = data

    def gps_sub_callback(self, data):
        self.data["GPS"] = data

    def lidar_sub_callback(self, data):
        self.data["LiDAR"] = data
        
    def risk_calculator_lidar_data_callback(self):
        kit.servo[0].angle = 130 # 
        kit.servo[1].angle = 130 # 
        point_groups = []
        points_tmp = []
        x_group = []
        y_group = []
        for data_single in self.data["LiDAR"].points :
            if data_single.x >= 0 and math.sqrt(math.pow(data_single.x,2) + math.pow(data_single.y,2)) < 1.5:
                x_group.append(data_single.x)
                y_group.append(data_single.y)
        # self.get_logger().info(
                # "x_group: {0}\n".format(
                    # x_group,
                    # ))                
        if(len(x_group) != 0):
            point_last_x = x_group[0]
            point_last_y = y_group[0]
            i_end = len(x_group)
            is_end_append = False    
        
            point_groups.append([[0,1.5]])
            for i in range(1,i_end):
                point_now_x = x_group[i] 
                point_now_y = y_group[i] 
                length_a = point_last_x - point_now_x
                length_b = point_last_y - point_now_y
                length_c = math.sqrt((length_a * length_a) + (length_b * length_b))
            
                if length_c < 0.07 : # 이전 점과 거리가 짧으면 임시 그룹에 넣음
                    points_tmp.append(copy.deepcopy([point_now_x,point_now_y]))
                else : # 이전 점에서 멀리 떨어져 있으면 다음 그룹의 점으로 간주하고 기존 그룹은 저장
                    if len(points_tmp) > 10 : # 구성이 10개보다 큰 경우에만 그룹으로 인정
                        if i == i_end - 1 :
                            is_end_append = True
                        point_groups.append(copy.deepcopy(points_tmp))
                    points_tmp = []
                point_last_x = copy.deepcopy(point_now_x)
                point_last_y = copy.deepcopy(point_now_y)
            if len(points_tmp) > 10 and is_end_append == False : # 마지막 그룹까지 계산
                point_groups.append(copy.deepcopy(points_tmp))
            point_groups.append([[0,-1.5]])
                        
                
            length = []                            
            for i in range(len(point_groups) - 1):
                 length.append([math.sqrt(math.pow((point_groups[i][-1][0] - point_groups[i+1][0][0]),2) + math.pow((point_groups[i][-1][1] - point_groups[i+1][0][1]),2)),i])                
            # self.get_logger().info(
                # "length: {0}\n".format(
                    # length,
                    # ))     
        
            can_pass = []
            for i in range(len(length)):
                if length[i][0] > 0.9:
                    can_pass.append(length[i][1])           # group index 
            # self.get_logger().info(
                # "can_pass: {0}\n".format(
                    # can_pass,
                    # ))
                    
                
            safe_angles = []
            # can pass over two way
            if len(can_pass) != 1:
                # if can_pass.count(0) == 1:
                    # safe_angles.append([math.atan2(1.5,0)*(180/math.pi) + 90,math.atan2(point_groups[can_pass[1]][0][1],point_groups[can_pass[1]][0][0])*(180/math.pi) + 90])
                for i in range(len(can_pass)-1): 
                    x1 = point_groups[can_pass[i]][-1][0]
                    x2 = point_groups[can_pass[i+1]][0][0]
                    y1 = point_groups[can_pass[i]][-1][1]
                    y2 = point_groups[can_pass[i+1]][0][1]
                    self.get_logger().info(
                      "x1: {0}\t y1: {1}\n x2: {2}\t y2: {3}\n len : {4}".format(
                      x1,x2,y1,y2,len(can_pass)
                    ))
                    # x_1 = x1 + (0.35 / length[can_pass[i]][0]) * (x2 - x1)
                    # y_1 = y1 + (0.35 / length[can_pass[i]][0]) * (y2 - y1)
                    # x_2 = x2 + (0.35 / length[can_pass[i]][0]) * (x2 - x1)
                    # y_2 = y2 + (0.35 / length[can_pass[i]][0]) * (y2 - y1)
                    # angle_1 = math.atan2(y_1,x_1)*(180/math.pi)
                    # angle_2 = math.atan2(y_2,x_2)*(180/math.pi)                       
                    # safe_angles.append([angle_1,angle_2])
                    x_m = (x1 + x2) / 2
                    y_m = (y1 + y2) / 2
                    angle = math.atan2(y_m,x_m)*(180/math.pi)
                    angle = abs(angle-90)
                    safe_angles.append(angle)
                # if can_pass.count(len(can_pass) - 1) == 1:
                    # safe_angles.append([math.atan2(point_groups[can_pass[len(can_pass)-2]][-1][1],point_groups[can_pass[len(can_pass)-2]][-1][0])*(180/math.pi) + 90,math.atan2(-1.5,0)*(180/math.pi) + 90])
           # only can pass one way
            else:
                self.get_logger().info("Only one pass existed!\n")             
                x1 = point_groups[can_pass[0]][-1][0]
                x2 = point_groups[can_pass[0]+1][0][0]
                y1 = point_groups[can_pass[0]][-1][1]
                y2 = point_groups[can_pass[0]+1][0][1]
                # x_1 = x1 + (0.35 / length[can_pass[0]][0]) * (x2 - x1)
                # y_1 = y1 + (0.35 / length[can_pass[0]][0]) * (y2 - y1)
                # x_2 = x2 + (0.35 / length[can_pass[0]][0]) * (x2 - x1)
                # y_2 = y2 + (0.35 / length[can_pass[0]][0]) * (y2 - y1)
                # angle_1 = math.atan2(y_1,x_1)*(180/math.pi)
                # angle_2 = math.atan2(y_2,x_2)*(180/math.pi)                
                # safe_angles.append([angle_1,angle_2])
                x_m = (x1 + x2) / 2
                y_m = (y1 + y2) / 2
                angle = math.atan2(y_m,x_m)*(180/math.pi)
                angle = abs(angle-90)
                safe_angles.append(angle)
                
            #sorting safe angle
        
            self.get_logger().info(
                "safe_angles: {0}\n###############\n".format(
                    safe_angles,
                    ))    
            
        
            # idle_angle = 90
            # if 0 < idle_angle <= 30:
                # self.kit.servo[0].angle = 70
                # self.kit.servo[1].angle = 100
            # elif 30 < idle_angle <= 60:
                # self.kit.servo[0].angle = 80
                # self.kit.servo[1].angle = 100
            # elif 60 < idle_angle <= 80:
                # self.kit.servo[0].angle = 90
                # self.kit.servo[1].angle = 100
            # elif 80 < idle_angle <= 90:
                # self.kit.servo[0].angle = 95
                # self.kit.servo[1].angle = 100
            # elif 90 < idle_angle <= 100:
                # self.kit.servo[0].angle = 100
                # self.kit.servo[1].angle = 95
            # elif 100 < idle_angle <= 120:
                # self.kit.servo[0].angle = 100
                # self.kit.servo[1].angle = 90
            # elif 120 < idle_angle <= 150:
                # self.kit.servo[0].angle = 100
                # self.kit.servo[1].angle = 80
            # elif 150 < idle_angle <= 180:
                # self.kit.servo[0].angle = 100
                # self.kit.servo[1].angle = 70
                              
        # # 그룹화
        # # point_groups = [] # 최종 점들 그룹
        # # points_tmp = [] # 최종 점들의 후보 점 임시 저장
        # # i_end = len(pointcloud_msg.points)
        # # point_now = Point32()
        # # point_last = Point32()
        # # is_end_append = False # 마지막 점이 포함된 그룹이 append되었는지 여부.
        # # point_last.x = pointcloud_msg.points[0].x
        # # point_last.y = pointcloud_msg.points[0].y
        # # for i in range(1, i_end) :
        # #     point_now.x = copy.deepcopy(pointcloud_msg.points[i].x)
        # #     point_now.y = copy.deepcopy(pointcloud_msg.points[i].y)

        # #     #  삼각함수 사용해 점간 거리 계산
        # #     length_a = point_last.x - point_now.x
        # #     length_b = point_last.y - point_now.y
        # #     length_c = math.sqrt((length_a * length_a) + (length_b * length_b))

        # #     if length_c < 0.1 : # 이전 점과 거리가 짧으면 임시 그룹에 넣음
        # #         points_tmp.append(copy.deepcopy(point_now))
        # #     else : # 이전 점에서 멀리 떨어져 있으면 다음 그룹의 점으로 간주하고 기존 그룹은 저장
        # #         if len(points_tmp) > 10 : # 구성이 10개보다 큰 경우에만 그룹으로 인정
        # #             if i == i_end - 1 :
        # #                 is_end_append = True
        # #             point_groups.append(copy.deepcopy(points_tmp))
        # #         points_tmp = []

        # #     point_last.x = copy.deepcopy(point_now.x)
        # #     point_last.y = copy.deepcopy(point_now.y)

        # # if len(points_tmp) > 10 and is_end_append == False : # 마지막 그룹까지 계산
        # #     point_groups.append(copy.deepcopy(points_tmp))
        
        
        # # before code
        
        
        # # self.get_logger().info("Send lidar data example")
        # # x_group = []
        # # y_group = []
        # # angle_group = []
        # # risk_group = []
        # # x_sum = 0
        # # y_sum = 0
        # # x_mean = 0
        # # y_mean = 0
        # # cnt = 0
        # # DISTANCE_MIN = 1.5
        # # angle 180 ~ -180
        # # angle_increment = 0.011 angle_max = 3.15 angle_min = - 3.15        
        # # risk_group = [0] * len(x_group)    
        
        # # for i in range(len(x_group)-1):
            # # if(x_group[i] <= 0):
                # # risk_group[i] += 10
                
            # # dist = math.sqrt(math.pow(x_group[i],2) + math.pow(y_group[i],2))  
            # # if(dist < DISTANCE_MIN):
                # # risk_group[i] += 1
            # # if(angle_group[i] * (180/math.pi) > 85 or angle_group[i] * (180/math.pi) < -85):
                # # risk_group[i] += 1    
             
                 
        # # angle_and_cnt = []
                
        # # for i in range(len(risk_group)-1):
            # # if(risk_group[i] == 0):
                # # x_sum += x_group[i]
                # # y_sum += y_group[i]
                # # cnt += 1
            # # else:
                # # if(cnt != 0):
                    # # x_mean = x_sum/cnt	
                    # # y_mean = y_sum/cnt
                    # # angle_mean = math.atan2(y_mean,x_mean)
                    # # angle_and_cnt.append([angle_mean * (180/math.pi),cnt])
                # # x_sum = 0
                # # y_sum = 0
                # # cnt = 0 
                # # Max = 0
        # # optimal_angle = 0  
        # # for i in range(len(angle_and_cnt)):
            # # self.get_logger().info(
            # # "\nangle: {0}".format(
                # # angle_and_cnt[i][0],
                # # angle_and_cnt[i][1],
            # # )
            # # )
            # # if(angle_and_cnt[i][1] > Max):
                # # MAX = angle_and_cnt
                # # optimal_angle = angle_and_cnt[i][0]      
            # # if(optimal_angle < 0):
                # # abs(optimal_angle)+90
                
            # # if(optimal_angle > 20):
                # # if(optimal_angle > 110):
                    # # optimal_angle = 110
                # # elif(optimal_angle > 90 and optimal_angle < 110):
                    # # optimal_angle = optimal_angle
                # # else:
                    # # optimal_angle = 20
            
            # # if(optimal_angle > 90):
                # # self.set_motors(100,90-optimal_angle)                       
            # # else:
                # # self.set_motors(100,optimal_angle+90)    

    def light_callback(self):
        if self.data["Shape"] != "None":
            if self.data["Shape"] == "plus":
                self.set_pixel(254,0,0)
            elif self.data["Shape"] == "circle":    
                self.set_pixel(0,254,0)
            elif self.data["Shape"] == "triangle":
                self.set_pixel(0,0,254)
       
			
				
    def set_motors(self, bldc_pwr, servo_deg):
        """
        쓰로틀과 키의 속도를 설정합니다.
        :param bldc_pwr: 쓰로틀의 속도. 0~100의 범위를 가지고 있습니다.
        :param servo_deg: 키의 각도. 0~180의 입력 범위를 가집니다.
        """
        data_actuator = Actuator.Request()
        data_actuator.throttle_pwr = bldc_pwr
        data_actuator.key_dgr = servo_deg
        self.actuator_set_handler.call_async(data_actuator)
      

    def set_pixel(self, _r, _g, _b):
        """
        네오픽셀 링의 R, G, B 값을 설정합니다. 만약 세 값이 동일하다면 RGB는 비활성화되며 흰색 LED가 설정한 밝기로작동합니다.
        :param r: 0~254 범위를 가지는 빨간색 데이터
        :param g: 0~254 범위를 가지는 초록색 데이터
        :param b: 0~254 범위를 가지는 파란색 데이터
        """ 
        _r = 0 if _r < 0 else _r
        _r = 254 if _r > 254 else _r
        _r = int(_r)

        _g = 0 if _g < 0 else _g
        _g = 254 if _g > 254 else _g
        _g = int(_g)

        _b = 0 if _b < 0 else _b
        _b = 254 if _b > 254 else _b
        _b = int(_b)

        data_rgb = Color.Request()
        data_rgb.red = _r
        data_rgb.green = _g
        data_rgb.blue = _b
        self.color_set_handler.call_async(data_rgb)
        


def main(args=None):
    rclpy.init(args=args)

    mecha_autoship_example_node = MechaAutoshipExampleNode()
    rclpy.spin(mecha_autoship_example_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
