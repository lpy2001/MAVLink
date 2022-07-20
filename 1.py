import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal,APIException
from pymavlink import mavutil
import math
import exceptions
import socket
import argparse
import serial
#���Ӵ���
def connect_serial():
    ser = serial.Serial('/dev/device0',9600)
    return ser

#�������˻�
def connect_vehicle():
    # Connect to the Vehicle.
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    connection_string = args.connect
    if not connection_string:
        print('��Ĭ�ϵĴ���%s���������˻�' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)
    return vehicle
#�������˻�
def arm():
    while vehicle.is_armable is False:
        print("�ȴ����˻�����")
        time.sleep(1)
    print("���˻����ڿ��Ա�����")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while vehicle.armed is False:
        print("�ȴ����˻�������")
        time.sleep(1)
    print("���˻��ѽ���")
#�ر����˻�
def close_vehicle(vehicle):
    vehicle.close()
    print("���˻��ر�")

#������
if __name__ == '__main__':
    vehicle = connect_vehicle()
    arm()
    time.sleep(5)
    vehicle.close()
    close_vehicle(vehicle)
    