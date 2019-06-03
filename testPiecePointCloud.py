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


os.chdir(wd + "\snaps1\Snaps")
scan_fname = getNpyPaths()
os.chdir(wd + "\snaps1\SnapPoses")
scan_pose_fname = getNpyPaths()
os.chdir(wd)
X = np.load("X.npy")

#load all scans and scan poses into arrays
scans = []
scan_poses = []
for fname in scan_fname:
    scans.append(np.load(fname))
scans = np.asarray(scans)
for fname in scan_pose_fname:
    scan_poses.append(np.load(fname))
scan_poses = np.asarray(scan_poses)

#gather all scans into a single pointcloud array
pointcloud = []
i = 0
for scan in scans:
    Y = np.dot(scan_poses[i],X)
    for point in scan:
        pointcloud.append(np.dot(Y,np.append(point,1)))
    i += 1 
points = unhomogenify(np.asarray(pointcloud))

"""
#rough filtering
Z_mean = np.mean(points[:,2])
Points = []
sigma = 0.05
for point in points:
    z = point[2]
    if z < Z_mean - sigma*Z_mean:
        pass
    elif z > Z_mean + sigma*Z_mean:
        pass
    else:
        Points.append(point)
points = np.asarray(Points)


#compute surface normals
surface_normals = []
startpoint = []
for i in range(0,len(points)-1):
    startpoint.append(points[i])
    surface_normals.append(np.cross(points[i],points[i+1]))
startpoint = np.asarray(startpoint)
surface_normals = np.asarray(surface_normals)
surface_normals /= np.linalg.norm(surface_normals)


#compute the mean of the surface normals
mean_normals = []
mean_points = []
s = 5 #compute the mean normal of every "s" points
for j in range(s,len(surface_normals),s):
    mean_points.append(np.array([np.mean(startpoint[j-s:j][:,0]), np.mean(startpoint[j-s:j][:,1]), np.mean(startpoint[j-s:j][:,2])]))
    n = np.array([np.mean(surface_normals[j-s:j][:,0]),np.mean(surface_normals[j-s:j][:,1]),np.mean(surface_normals[j-s:j][:,2])])
    n /= np.linalg.norm(n)
    mean_normals.append(n)
mean_points = np.asarray(mean_points)
mean_normals = np.asarray(mean_normals)

#find the inner product between the normals to identify point of change, where a new plane should be defined.
DOTS = []
for k in range(0,len(mean_normals)-1):
    DOT = np.linalg.norm(np.rad2deg(np.arccos(np.dot(mean_normals[k],mean_normals[k+1])/(np.linalg.norm(mean_normals[k])*np.linalg.norm(mean_normals[k+1])))))
    DOTS.append(DOT)
    print(DOT)
DOTS = np.asarray(DOTS)

change_points = []
change_index = []
for l in range(0,len(DOTS)-1):
    if np.abs((DOTS[l] - DOTS[l+1])) > 30:
        change_index.append(l)
        change_points.append(mean_points[l])

new_index = []
for h in range(1,len(change_index)):
    if np.abs(change_index[h] - change_index[h-1]) > 10:
        new_index.append(change_index[h])
"""
#change_points = np.asarray(change_points)
#change_points = mean_points[new_index]
#print(change_points.shape[0])
#fit1,c,err,outliers1 = ransacPlane(mean_points)
#fit2,c,err,outliers2 = ransacPlane(outliers1)
#fit3,c,err,outliers3 = ransacPlane(outliers2)
#fit4,c,err,outliers4 = ransacPlane(outliers3)
#print(np.around(fit1,decimals = 4))
#print(np.around(fit2,decimals = 4))
#print(np.abs(fit1[3]-fit2[3]))
#print(np.around(fit3,decimals = 2))
#print(np.around(fit4,decimals = 2))

#points = change_points
x = points[:,0]
y = points[:,1]
z = points[:,2]
plt.figure(1)
ax = plt.subplot(111, projection ='3d')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
#ax.set_zlim(250,350)
#ax.quiver(mean_points[:,0],mean_points[:,1],mean_points[:,2],mean_normals[:,0],mean_normals[:,1],mean_normals[:,2],length = 5, normalize = True)
ax.scatter(x, y, z, color ='r')
#plotPlane(fit1,ax,"g",0.3)
#plotPlane(fit2,ax,"y",0.7)
#plotPlane(fit3,ax,"b",1)
#plotPlane(fit4,ax,"y",1)
plt.show()