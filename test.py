import os
import re
import random
import numpy as np
import glob
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cv2 as cv2

np.set_printoptions(suppress=True)
pi = np.pi
c = np.cos
s = np.sin
transpose = np.transpose
sqrt = np.sqrt

def homogenify(G):
    H = []
    for point in G:
        H.append(np.append(point,1))
    return np.asarray(H)

def getUname():
    uname = os.getlogin()
    if uname == "trymdh":
        uname = uname + ".WIN-NTNU-NO"
    return uname
def sortList(unsortedList):
    #sort a list in alphanumeric order
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)',key)]
    return sorted(unsortedList,key = alphanum_key)
    
points = np.load("POOOOOOINTS.npy")
base_tool = np.load("T.npy")
base_tool[0:3,3] = base_tool[0:3,3]*1000
fig = plt.figure(1)
X = np.load("X.npy")
Y = np.linalg.inv(np.dot(base_tool,X))
print(Y)
points_Y = []
for point in points:
    points_Y.append(np.dot(Y,np.append(point,1)))
points_Y = np.asarray(points_Y)
ax = plt.axes(projection='3d')
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
#clean_points = points
X = points_Y[:,0]
Y = points_Y[:,1]
Z = points_Y[:,2]
ax.set_zlim3d(0,max(Z))
ax.set_ylim3d(0,max(Y))
ax.set_xlim3d(0,max(X))
print(max(Z)-min(Z))
ax.scatter3D(X,Y,Z)
plt.show()