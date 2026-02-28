import numpy as np
import math

def calc_rot_mtx(yaw, pitch, roll):
    R = np.array([[math.cos(pitch)*math.cos(roll), math.sin(yaw)*math.sin(pitch)*math.cos(roll) - math.cos(yaw)*math.sin(roll), math.cos(yaw)*math.sin(pitch)*math.cos(roll) + math.sin(yaw)*math.sin(roll)],
                  [math.cos(pitch)*math.sin(roll), math.sin(yaw)*math.sin(pitch)*math.sin(roll) + math.cos(yaw)*math.cos(roll), math.cos(yaw)*math.sin(pitch)*math.sin(roll) - math.sin(yaw)*math.cos(roll)],
                  [-math.sin(pitch), math.sin(yaw)*math.cos(pitch), math.cos(yaw)*math.cos(pitch)]])
    
    return R

R = calc_rot_mtx(math.pi/2, 0, 0)

T = np.identity(4, dtype = float)

T[:3, :3] = R

T[0, 3] = 4.625594
T[1, 3] = 3.43789
T[2, 3] = 1.124

print(T)