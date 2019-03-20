from AT_cx_functions import*
import cx.cx_cam as cam
import cx.cx_base as base 
import time 
from matplotlib import pyplot as plt
import cv2
import os
import glob

wd = getCurrentWD()
laserimage_folder = wd + "/laserimage"
os.chdir(laserimage_folder) 


laser = np.load("pixcoord_1.npy")
for point in laser:
    point = np.append(point,1)
    print(point[1])
plt.gca().invert_yaxis()
plt.plot(laser[:,1])
plt.show()