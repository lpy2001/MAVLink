import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal,APIException
from pymavlink import mavutil
import math
import exceptions
import socket
import argparse
import serial
#连接串口
def connect_serial():
    ser = serial.Serial('/dev/device0',9600)
    return ser

#连接无人机
def connect_vehicle():
    # Connect to the Vehicle.
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    connection_string = args.connect
    if not connection_string:
        print('在默认的串口%s上连接无人机' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)
    return vehicle
#启动无人机
def arm():
    while vehicle.is_armable is False:
        print("等待无人机启动")
        time.sleep(1)
    print("无人机现在可以被控制")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while vehicle.armed is False:
        print("等待无人机被解锁")
        time.sleep(1)
    print("无人机已解锁")
#关闭无人机
def close_vehicle(vehicle):
    vehicle.close()
    print("无人机关闭")

#主函数
if __name__ == '__main__':
    vehicle = connect_vehicle()
    arm()
    time.sleep(5)
    vehicle.close()
    close_vehicle(vehicle)
    