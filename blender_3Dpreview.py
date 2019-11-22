import bpy
import pandas as pd
import math
import csv

df = pd.read_csv('completion_data.csv')
df = df.drop(df.index[0], axis=0)
df = df.reset_index(drop=True)
df = df.astype('float64')

ob = bpy.data.objects["Rocket"]
ob.rotation_mode = 'QUATERNION'
ob.rotation_quaternion = (1, 0, 0, 0)
frame_num = 0

for i in range(df.shape[0]):
    frame_num += 10
    bpy.context.scene.frame_set(frame_num)

    x_difference = df.iloc[i, 9]
    y_difference = df.iloc[i, 10]
    z_difference = df.iloc[i, 11]
    position = (x_difference, y_difference, z_difference)
    ob.location = position

    w = df.iloc[i, 6]
    x = df.iloc[i, 7]
    y = df.iloc[i, 8]
    z = df.iloc[i, 9]
    quaternion = (w, x, y, z)
    ob.rotation_quaternion = quaternion
    ob.keyframe_insert(data_path="location", index = -1)
    ob.keyframe_insert(data_path="rotation_quaternion", index = -1)