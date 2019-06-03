import numpy as np
from numpy import pi,cos,sin,tan,dot
import os
import glob
import re
from UtilityFunctions import getUname, sortList
np.set_printoptions(suppress=True)

uname = getUname()
os.chdir("C:/Users/" + uname +"/Onedrive/tpk4940Master/snaps/SnapPoses")
T_pose_list = sortList(glob.glob(os.getcwd().replace("\\","/") + "/*.npy"))
os.chdir("C:/Users/" + uname +"/Onedrive/tpk4940Master/snaps1/SnapPoses")
T1_pose_list = sortList(glob.glob(os.getcwd().replace("\\","/") + "/*.npy"))

T = []
for fname in T_pose_list:
    T.append(np.load(fname))
T1 = []
for fname in T1_pose_list:
    T1.append(np.load(fname))

print(T[1])
print(T1[1])