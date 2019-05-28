
import os
import re
import random
import numpy as np
import glob
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cv2

np.set_printoptions(suppress=True)
pi = np.pi
c = np.cos
s = np.sin
transpose = np.transpose
sqrt = np.sqrt

def getUname():
    uname = os.getlogin()
    if uname == "trymdh":
        uname = uname + ".WIN-NTNU-NO"
    return uname
points = np.load("POOOOOOINTS.npy")
base_tool = np.load("T.npy")
base_tool[0:3,3] = base_tool[0:3,3]*1000
print(base_tool)
fig = plt.figure(1)
X = np.load("X.npy")
Y = base_tool@X
print(Y)
points_Y = []
for j in range(0,300):
    #simulating movement along X axes
    Y[:3,3][0] += 0.1
    #Y[:3,3][2] += 0.1
    for point in points:
        points_Y.append(np.dot(Y,np.append(point,1)))
points_Y = np.asarray(points_Y)
points_Y = points_Y[::50]

#clean_points = points
X = points_Y[:,0]
Y = points_Y[:,1]
Z = points_Y[:,2]

ax = plt.axes(projection='3d')
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
#ax.set_aspect("equal")
ax.set_zlim3d(np.mean(Z)-50,np.mean(Z)+50)
ax.set_ylim3d(np.mean(Y)-50,np.mean(Y)+50)
ax.set_xlim3d(np.mean(X)-50,np.mean(X)+50)
ax.scatter3D(X,Y,Z)
plt.show()