import os
import re
import random
import numpy as np
import glob
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
import cv2
from stl import Mesh

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
print(points.shape)
points = points[700:1200]
base_tool = np.load("T.npy")
base_tool[0:3,3] = base_tool[0:3,3]*1000
print(base_tool)
X = np.load("X.npy")
Y = base_tool@X
print(Y)
points_Y = []
for j in range(0,100):
    #simulating movement along X axes
    Y[:3,3][0] += 1
    #Y[:3,3][2] += 0.1
    for point in points:
        points_Y.append(np.dot(Y,np.append(point,1)))
points_Y = np.asarray(points_Y)
points_Y = points_Y[::10]

#clean_points = points
X = points_Y[:,0]
Y = points_Y[:,1]
Z = points_Y[:,2]

# Load the STL files and add the vectors to the plot
os.chdir(os.getcwd() + "\TestPiece")
your_mesh = Mesh.from_file('Testpiece.stl')
#align the stl file with the point cloud for visual check
your_mesh.rotate([0,1,0],np.deg2rad(90))
your_mesh.rotate([1,0,0],np.deg2rad(271.25))
your_mesh.translate([280,-1122,299])

fig = plt.figure(1)
# Auto scale to the mesh size


ax = plt.axes(projection='3d')
ax.add_collection3d(mplot3d.art3d.Poly3DCollection(your_mesh.vectors))
scale = your_mesh.points.flatten(-1)
#ax.auto_scale_xyz(scale, scale, scale)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_aspect("equal")
s = 50
ax.set_zlim3d(np.mean(Z)-s,np.mean(Z)+s)
ax.set_ylim3d(np.mean(Y)-s,np.mean(Y)+s)
ax.set_xlim3d(np.mean(X)-s,np.mean(X)+s)
ax.scatter3D(X,Y,Z,color = "red",alpha = 0.5)
plt.show()

