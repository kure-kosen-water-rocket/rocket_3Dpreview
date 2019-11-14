import pandas as pd
import math
import csv
import warnings
import numpy as np
#import transformation
from numpy.linalg import norm
from quaternion import kuaternion
#from geometry_msgs.msg import Vector3
from pyquaternion import Quaternion

def sin(radian):
    return(math.sin(radian))

def cos(radian):
    return(math.cos(radian))

#def quaternion_to_euler(quaternion):
 #   e = transformations.euler_from_quaternion((quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
  #  return Vector3(x=e[0], y=e[1], z=e[2])

class MadgwickAHRS:
    samplePeriod = 1/256
    quaternion = kuaternion(1, 0, 0, 0)
    beta = 1

    def __init__(self, sampleperiod=None, quaternion=None, beta=None):
        """
        Initialize the class with the given parameters.
        :param sampleperiod: The sample period
        :param quaternion: Initial quaternion
        :param beta: Algorithm gain beta
        :return:
        """
        if sampleperiod is not None:
            self.samplePeriod = sampleperiod
        if quaternion is not None:
            self.quaternion = quaternion
        if beta is not None:
            self.beta = beta


    def update_imu(self, gyroscope, accelerometer):
        """
        Perform one update step with data from a IMU sensor array
        :param gyroscope: A three-element array containing the gyroscope data in radians per second.
        :param accelerometer: A three-element array containing the accelerometer data. Can be any unit since a normalized value is used.
        """
        q = self.quaternion

        gyroscope = np.array(gyroscope, dtype=float).flatten()
        accelerometer = np.array(accelerometer, dtype=float).flatten()

        # Normalise accelerometer measurement
        if norm(accelerometer) is 0:
            warnings.warn("accelerometer is zero")
            return
        accelerometer /= norm(accelerometer)

        # Gradient descent algorithm corrective step
        f = np.array([
            2*(q[1]*q[3] - q[0]*q[2]) - accelerometer[0],
            2*(q[0]*q[1] + q[2]*q[3]) - accelerometer[1],
            2*(0.5 - q[1]**2 - q[2]**2) - accelerometer[2]
        ])
        j = np.array([
            [-2*q[2], 2*q[3], -2*q[0], 2*q[1]],
            [2*q[1], 2*q[0], 2*q[3], 2*q[2]],
            [0, -4*q[1], -4*q[2], 0]
        ])
        step = j.T.dot(f)
        step /= norm(step)  # normalise step magnitude

        # Compute rate of change of quaternion
        qdot = (q * kuaternion(0, gyroscope[0], gyroscope[1], gyroscope[2])) * 0.5 - self.beta * step.T

        # Integrate to yield quaternion
        q += qdot * self.samplePeriod
        self.quaternion = kuaternion(q / norm(q))  # normalise quaternion

        quaternion = self.quaternion
        r, p, y = kuaternion.to_euler123(quaternion)
        return(quaternion[0], quaternion[1], quaternion[2], quaternion[3], r, p, y)

with open('calculation.csv','w') as calculation_file:                            #'measurement.csv'というファイルを作成する
        field = ['x_difference','y_difference','z_difference']
        writer = csv.DictWriter(calculation_file, fieldnames=field)
        writer.writeheader()

df = pd.read_csv('measurement.csv') #左からx,y,z,y_angle
df = df.astype('float64')
print(df.head())

angle_df = pd.DataFrame(np.random.random([df.shape[0],7]), columns=['w','x','y','z','roll','pitch','yaw'])
print(angle_df.head())
angle_df.iloc[0,0] = 2000
print(angle_df.head())

dt = 0.01
x_old_accel = 0
y_old_accel = 0
z_old_accel = 0

x_old_speed = 0
y_old_speed = 0
z_old_speed = 0

x_difference = 0
y_difference = 0
z_difference = 0

x_old_difference = 0
y_old_difference = 0
z_old_difference = 0

gyroscope     = [0, 0, 0]
accelerometer = [0, 0, 0]

md = MadgwickAHRS()

for i in range(df.shape[0]):
    accelerometer = [df.iloc[i, 0], df.iloc[i, 1], df.iloc[i, 2]]
    gyroscope     = [df.iloc[i, 3], df.iloc[i, 4], df.iloc[i, 5]]

    w, x, y, z, r, p, y = md.update_imu(gyroscope, accelerometer)
    quaterion  = np.array([w, x, y, z])

    #print(r)
    roll  = r
    pitch = p
    yaw   = y
    #print(round(math.degrees(roll), 0),"do")

    angle_df.iloc[i, 0] = w
    angle_df.iloc[i, 1] = x
    angle_df.iloc[i, 2] = y
    angle_df.iloc[i, 3] = z
    angle_df.iloc[i, 4] = roll
    angle_df.iloc[i, 5] = pitch
    angle_df.iloc[i, 6] = yaw

for i in range(df.shape[0]):
    x_gravity_acceleration   = 0.980665 * cos(angle_df.iloc[i, 5])
    y_gravity_acceleration   = 0.980665 * cos(angle_df.iloc[i, 4])
    z_gravity_acceleration   = 0.980665 * cos(angle_df.iloc[i, 5])

    x_motion_acceleration    = df.iloc[i, 0] - x_gravity_acceleration
    y_motion_acceleration    = df.iloc[i, 1] - y_gravity_acceleration
    z_motion_acceleration    = df.iloc[i, 2] - z_gravity_acceleration

    #x_change_direction_accel = x_motion_acceleration * math.cos(df.iloc[i, 11])
    #y_change_direction_accel = y_motion_acceleration * math.cos(df.iloc[i, 10])
    #z_change_direction_accel = z_motion_acceleration * math.cos(df.iloc[i, 11])

    x_speed = (((x_motion_acceleration + x_old_accel) * dt) / 2) #+ x_old_speed
    y_speed = (((y_motion_acceleration + y_old_accel) * dt) / 2) #+ y_old_speed
    z_speed = (((z_motion_acceleration + z_old_accel) * dt) / 2) #+ z_old_speed

    x_difference = (((x_speed + x_old_speed) * dt) / 2) #+ x_old_difference
    y_difference = (((y_speed + y_old_speed) * dt) / 2) #+ y_old_difference
    z_difference = (((z_speed + z_old_speed) * dt) / 2) #+ z_old_difference

    x_old_accel = x_motion_acceleration #x_change_direction_accel
    y_old_accel = y_motion_acceleration #y_change_direction_accel
    z_old_accel = z_motion_acceleration #z_change_direction_accel

    x_old_speed = x_speed
    y_old_speed = y_speed
    z_old_speed = z_speed

    x_total_speed = x_speed + x_old_speed
    y_total_speed = y_speed + y_old_speed
    z_total_speed = z_speed + z_old_speed

    x_old_difference += x_difference
    y_old_difference += y_difference
    z_old_difference += z_difference

    with open('calculation.csv', 'a') as calculation_file:
        writer = csv.DictWriter(calculation_file, fieldnames=field)
        writer.writerow({'x_difference':x_difference,
                         'y_difference':y_difference,
                         'z_difference':z_difference})

df_2 = pd.read_csv('calculation.csv')
df_2 = df_2.astype('float64')
print(df_2.head())

df_3 = pd.concat([df  , angle_df], axis=1)
df_4 = pd.concat([df_3, df_2], axis=1)
print(df_4.head())


df_4.to_csv('completion_data.csv')

print("[x_old_difference]",x_old_difference,"　", "[y_old_difference]",y_old_difference,"　", "[z_old_difference]","　" ,z_old_difference)
