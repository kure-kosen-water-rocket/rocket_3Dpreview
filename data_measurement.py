import smbus
import math
from time import sleep
import signal
import csv
#import warnings
#import numpy as np
#from numpy.linalg import norm
#from quaternion import Quaternion

# slaveaddress
DEV_ADDR = 0x68         # device address
# register address
ACCEL_XOUT = 0x3b
ACCEL_YOUT = 0x3d
ACCEL_ZOUT = 0x3f
TEMP_OUT = 0x41
GYRO_XOUT = 0x43
GYRO_YOUT = 0x45
GYRO_ZOUT = 0x47
PWR_MGMT_1 = 0x6b       # PWR_MGMT_1
PWR_MGMT_2 = 0x6c       # PWR_MGMT_2

bus = smbus.SMBus(1)
bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 0)

def read_byte(adr):
    return bus.read_byte_data(DEV_ADDR, adr)
    
def read_word(adr):
    high = bus.read_byte_data(DEV_ADDR, adr)
    low  = bus.read_byte_data(DEV_ADDR, adr+1)
    val  = (high << 8) + low
    return val

def read_word_sensor(adr):
    val  = read_word(adr)

    if (val >= 0x8000):
        return -((65535 - val) + 1)

    else:
        return val

def get_gyro_data_lsb():               #角速度(ジャイロ)データ取得
    x = read_word_sensor(GYRO_XOUT)
    y = read_word_sensor(GYRO_YOUT)
    z = read_word_sensor(GYRO_ZOUT)
    return [x, y, z]

def get_gyro_data_deg():
    x,y,z = get_gyro_data_lsb()
    x = x / 1310
    y = y / 1310
    z = z / 1310
    return [x, y, z]

def get_accel_data_lsb():              #加速度データ取得
    x = read_word_sensor(ACCEL_XOUT)
    y = read_word_sensor(ACCEL_YOUT)
    z = read_word_sensor(ACCEL_ZOUT)
    return [x, y, z]

def get_accel_data_g():
    x,y,z = get_accel_data_lsb()
    x = ((x / 16384.0) *1)            # *10 => m/sに変換
    y = ((y / 16384.0) *1)
    z = ((z / 16384.0) *1)
    return [x, y, z]

with open('measurement.csv','w') as measurement_file:                            #'measurement.csv'というファイルを作成する
        fieldnames = ['x_accel','y_accel','z_accel','x_gyro','y_gyro','z_gyro']
        writer = csv.DictWriter(measurement_file, fieldnames=fieldnames)
        writer.writeheader()

#get_init_angle_data()

time = 0
dt   = 0.01
calculate_time = 10
gyroscope     = [0, 0, 0]
accelerometer = [0, 0, 0]

#md = MadgwickAHRS()
while 1:
    x_gyro,  y_gyro,  z_gyro  = get_gyro_data_deg()

    x_accel, y_accel, z_accel = get_accel_data_g()
    print(x_accel)

    #gyroscope     = [x_gyro , y_gyro , z_gyro]
    #accelerometer = [x_accel, y_accel, z_accel]

    #w, x, y, z = md.update_imu(gyroscope, accelmeter)
    with open('measurement.csv', 'a') as measurement_file:
        writer = csv.DictWriter(measurement_file, fieldnames=fieldnames)
        writer.writerow({'x_accel':x_accel,
                         'y_accel':y_accel,
                         'z_accel':z_accel,
                         'x_gyro' :x_gyro ,
                         'y_gyro' :y_gyro ,
                         'z_gyro' :z_gyro ,})

    time += dt

    if time > calculate_time:
        break

    sleep(dt)
