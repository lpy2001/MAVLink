import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal,APIException
from pymavlink import mavutil
import math
import exceptions
import socket
import argparse
import serial

#��������
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
#���ӷɿ�
vehicle = connect('/dev/ttyACM0', wait_ready=True)
#���÷ɿ�ģʽ
vehicle.mode = VehicleMode("GUIDED")
#���÷ɿص�Ŀ��λ��
vehicle.location.global_relative_frame = LocationGlobalRelative(0, 0, 0)
#���÷ɿص�Ŀ���ٶ�
vehicle.airspeed = 5
#���÷ɿص�Ŀ�꺽��
vehicle.heading = 0
#���÷ɿص�Ŀ��߶�
vehicle.location.global_relative_frame.alt = 0
#���÷ɿص�Ŀ�꾭��
vehicle.location.global_relative_frame.lon = 0
#���÷ɿص�Ŀ��γ��
vehicle.location.global_relative_frame.lat = 0
#У�鶯���Ƿ�����ӳٺ���
def delay(t):
    time.sleep(t)
    print("Delay")
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt <= 0.1:
            print("Reached target altitude")
            break
        time.sleep(1)

#�ɿ����
def arm_and_takeoff(aTargetAltitude):
    print("����ǰ���")
    # Wait until the vehicle is ready to arm, aka is not busy
    while not vehicle.is_armable:
        print("�ȴ����˻�����")
        time.sleep(1)
    print("�綯������")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    # Wait until the vehicle is armed
    while not vehicle.armed:
        print("�ȴ����˻�������")
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
#�ɿؽ���
def land():
    print("Landing")
    vehicle.mode = VehicleMode("LAND")
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt <= 0.1:
            print("Reached target altitude")
            break
        time.sleep(1)
#�ɿؽ���
def end():
    print("Ending")
    vehicle.mode = VehicleMode("RTL")
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt <= 0.1:
            print("Reached target altitude")
            break
        time.sleep(1)
#�ɿؿ���
def control():
    print("Control")
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt <= 0.1:
            print("Reached target altitude")
            break
        time.sleep(1)

#������
def main():
    #���
    arm_and_takeoff(10)
    #�ȴ�10��
    delay(10)
    #ǰ��10��
    vehicle.commands.go_to(vehicle.location.global_relative_frame.lat + 0.0001, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt)
    #�ȴ�10��
    delay(10)
    #����10��
    vehicle.commands.go_to(vehicle.location.global_relative_frame.lat - 0.0001, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt)
    #�ȴ�10��
    delay(10)
    #��ת90��
    vehicle.commands.go_to(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon - 0.0001, vehicle.location.global_relative_frame.alt)
    #�ȴ�5��
    delay(5)
    #ǰ��10��
    vehicle.commands.go_to(vehicle.location.global_relative_frame.lat + 0.0001, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt)
    #�ȴ�10��
    delay(10)
    #��ת90��
    vehicle.commands.go_to(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon + 0.0001, vehicle.location.global_relative_frame.alt)
    #�ȴ�5��
    delay(5)
    #����10��
    vehicle.commands.go_to(vehicle.location.global_relative_frame.lat - 0.0001, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt)
    #�ȴ�10��
    delay(10)
    #�ص�ԭ��
    vehicle.commands.go_to(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt)
    #�ȴ�10��
    delay(10)
    #����
    land()
    #����
    end()
#������
if __name__ == '__main__':
    #�򿪴���
    ser.open()
    #������
    main()
    #�رշɿ�
    vehicle.close()
    #�رմ���
    ser.close()
