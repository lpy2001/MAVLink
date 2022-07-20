import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal,APIException
from pymavlink import mavutil
import math
import exceptions
import socket
import argparse
import serial

#串口连接
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
#连接飞控
vehicle = connect('/dev/ttyACM0', wait_ready=True)
#设置飞控模式
vehicle.mode = VehicleMode("GUIDED")
#设置飞控的目标位置
vehicle.location.global_relative_frame = LocationGlobalRelative(0, 0, 0)
#设置飞控的目标速度
vehicle.airspeed = 5
#设置飞控的目标航向
vehicle.heading = 0
#设置飞控的目标高度
vehicle.location.global_relative_frame.alt = 0
#设置飞控的目标经度
vehicle.location.global_relative_frame.lon = 0
#设置飞控的目标纬度
vehicle.location.global_relative_frame.lat = 0
#校验动作是否完成延迟函数
def delay(t):
    time.sleep(t)
    print("Delay")
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt <= 0.1:
            print("Reached target altitude")
            break
        time.sleep(1)

#飞控起飞
def arm_and_takeoff(aTargetAltitude):
    print("启动前检查")
    # Wait until the vehicle is ready to arm, aka is not busy
    while not vehicle.is_armable:
        print("等待无人机启动")
        time.sleep(1)
    print("电动机启动")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    # Wait until the vehicle is armed
    while not vehicle.armed:
        print("等待无人机被解锁")
        time.sleep(1)
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)
    # Wait until the vehicle reaches a safe height
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
#飞控降落
def land():
    print("Landing")
    vehicle.mode = VehicleMode("LAND")
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt <= 0.1:
            print("Reached target altitude")
            break
        time.sleep(1)
#飞控结束
def end():
    print("Ending")
    vehicle.mode = VehicleMode("RTL")
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt <= 0.1:
            print("Reached target altitude")
            break
        time.sleep(1)
#飞控控制
def control():
    print("Control")
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt <= 0.1:
            print("Reached target altitude")
            break
        time.sleep(1)

#主函数
def main():
    #起飞
    arm_and_takeoff(10)
    #等待10秒
    delay(10)
    #前进10米
    vehicle.commands.go_to(vehicle.location.global_relative_frame.lat + 0.0001, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt)
    #等待10秒
    delay(10)
    #后退10米
    vehicle.commands.go_to(vehicle.location.global_relative_frame.lat - 0.0001, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt)
    #等待10秒
    delay(10)
    #左转90度
    vehicle.commands.go_to(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon - 0.0001, vehicle.location.global_relative_frame.alt)
    #等待5秒
    delay(5)
    #前进10米
    vehicle.commands.go_to(vehicle.location.global_relative_frame.lat + 0.0001, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt)
    #等待10秒
    delay(10)
    #右转90度
    vehicle.commands.go_to(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon + 0.0001, vehicle.location.global_relative_frame.alt)
    #等待5秒
    delay(5)
    #后退10米
    vehicle.commands.go_to(vehicle.location.global_relative_frame.lat - 0.0001, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt)
    #等待10秒
    delay(10)
    #回到原点
    vehicle.commands.go_to(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt)
    #等待10秒
    delay(10)
    #降落
    land()
    #结束
    end()
#主函数
if __name__ == '__main__':
    #打开串口
    ser.open()
    #主函数
    main()
    #关闭飞控
    vehicle.close()
    #关闭串口
    ser.close()
