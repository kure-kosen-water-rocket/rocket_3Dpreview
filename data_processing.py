import pandas as pd
import math
import csv
import warnings
import numpy as np
from numpy.linalg import norm
from quaternion import kuaternion
from pyquaternion import Quaternion

def sin(radian):
    return(math.sin(radian))

def cos(radian):
    return(math.cos(radian))

class MadgwickAHRS:
    samplePeriod = 1/256
    quaternion = kuaternion(1, 0, 0, 0)
    beta = 1

    def __init__(self, sampleperiod=None, quaternion=None, beta=None):
        if sampleperiod is not None:
            self.samplePeriod = sampleperiod
        if quaternion is not None:
            self.quaternion = quaternion
        if beta is not None:
            self.beta = beta


    def update_imu(self, gyroscope, accelerometer):
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
            2*(q[1]*q[3] - q[0]*q[2])   - accelerometer[0],
            2*(q[0]*q[1] + q[2]*q[3])   - accelerometer[1],
            2*(0.5 - q[1]**2 - q[2]**2) - accelerometer[2]
        ])
        j = np.array([
            [-2*q[2], 2*q[3]  ,-2*q[0], 2*q[1]],
            [2*q[1] , 2*q[0]  , 2*q[3], 2*q[2]],
            [0      ,-4*q[1]  ,-4*q[2],      0]
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

with open('calculation.csv','w') as calculation_file:                            #'calculation.csv'というファイルを作成する
        field = ['x_difference','y_difference','z_difference']
        writer = csv.DictWriter(calculation_file, fieldnames=field)
        writer.writeheader()

df       = pd.read_csv('measurement.csv') #左からx,y,z,y_angle
df       = df.astype('float64')

angle_df = pd.DataFrame(np.random.random([df.shape[0],7]), columns=['w','x','y','z','roll','pitch','yaw'])

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

    roll  = r
    pitch = p
    yaw   = y

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

    x_speed = (((x_motion_acceleration + x_old_accel) * dt) / 2)
    y_speed = (((y_motion_acceleration + y_old_accel) * dt) / 2)
    z_speed = (((z_motion_acceleration + z_old_accel) * dt) / 2)

    x_difference = (((x_speed + x_old_speed) * dt) / 2)
    y_difference = (((y_speed + y_old_speed) * dt) / 2)
    z_difference = (((z_speed + z_old_speed) * dt) / 2)

    x_old_accel = x_motion_acceleration
    y_old_accel = y_motion_acceleration
    z_old_accel = z_motion_acceleration

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
df_3 = pd.concat([df  , angle_df], axis=1)
df_4 = pd.concat([df_3,     df_2], axis=1)
df_4 = df_4.astype('float64')

df_4.to_csv('completion_data.csv')

print("[x_old_difference]",x_old_difference)
print("[y_old_difference]",y_old_difference)
print("[z_old_difference]",z_old_difference)
