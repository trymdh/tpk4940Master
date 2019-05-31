import os
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from UtilityFunctions import getUname, getNpyPaths
from PointArrayFunctions import unhomogenify, homogenify
from RansacFunctions import ransacPlane
from PlaneFunctions import plotPlane
uname = getUname()
wd = os.getcwd()


os.chdir(wd + "\snaps")
scan_fname = getNpyPaths()
os.chdir(os.getcwd() + "\SnapPoses")
scan_pose_fname = getNpyPaths()
os.chdir(wd)
X = np.load("X.npy")

scans = []
scan_poses = []
for fname in scan_fname:
    scans.append(np.load(fname))
scans = np.asarray(scans)
for fname in scan_pose_fname:
    scan_poses.append(np.load(fname))
scan_poses = np.asarray(scan_poses)

pointcloud = []
i = 0
for scan in scans:
    Y = np.dot(scan_poses[i],X)
    for point in scan:
        pointcloud.append(np.dot(Y,np.append(point,1)))
    i += 1 

points = unhomogenify(np.asarray(pointcloud))

print(np.mean(points[:,0]),np.mean(points[:,1]),np.mean(points[:,2]))

"""
fit1,c,err,outliers1 = ransacPlane(points)
fit2,c,err,outliers2 = ransacPlane(outliers1)
#fit3,c,err,outliers3 = ransacPlane(outliers2)
#fit4,c,err,outliers4 = ransacPlane(outliers3)
print(np.around(fit1,decimals = 2))
print(np.around(fit2,decimals = 2))
#print(np.around(fit3,decimals = 2))
#print(np.around(fit4,decimals = 2))
"""
x = points[:,0]
y = points[:,1]
z = points[:,2]
plt.figure(1)
ax = plt.subplot(111, projection ='3d')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.scatter(x, y, z, color ='b')
#plotPlane(fit1,ax,"g",1)
#plotPlane(fit2,ax,"r",1)
#plotPlane(fit3,ax,"b",1)
#plotPlane(fit4,ax,"y",1)
plt.show()