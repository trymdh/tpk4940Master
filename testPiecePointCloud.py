import os
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from UtilityFunctions import getUname, getNpyPaths
from PointArrayFunctions import unhomogenify, homogenify
from RansacFunctions import ransacPlane
from PlaneFunctions import plotPlane

def filterPoints(points):
    outz = []
    for point in points:
        x,y,z = point
        if (np.mean(points[:,2]) - 1.5*np.std(points[:,2])) < z < (np.mean(points[:,2]) + 1.5*np.std(points[:,2])):
            outz.append(point)
    outy = []
    for point in outz:
        x,y,z = point
        if (np.mean(points[:,1]) - 1.5*np.std(points[:,1])) < y < (np.mean(points[:,1]) + 1.5*np.std(points[:,1])):
            outy.append(point)
    outx = []
    for point in outy:
        x,y,z = point
        if (np.mean(points[:,0]) - 1.5*np.std(points[:,0])) < x < (np.mean(points[:,0]) + 1.5*np.std(points[:,0])):
            outx.append(point)
    out = np.asarray(outx)
    print("{0} points where input and {1} where outputed".format(len(points),len(out)))
    return out

uname = getUname()
wd = os.getcwd()
"""
    os.chdir(wd + "\snaps\Snaps")
    scan_fname = getNpyPaths()
    os.chdir(wd + "\snaps\SnapPoses")
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

    for scan in scan_poses:
            scan[:3,3] = scan[:3,3]*1000

    #gather all scans into a single pointcloud array
    pointcloud = []
    i = 0
    for scan in scans:
        Y = np.dot(scan_poses[i],X)
        for point in scan:
            pointcloud.append(np.dot(Y,np.append(point,1)))
        i += 1 
    points = unhomogenify(np.asarray(pointcloud))
    points = filterPoints(points)
    os.chdir(wd)
    np.save("POINTCLOUD.npy",points)
"""

os.chdir(wd)
points = np.load("POINTCLOUD.npy")
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
s = 10 #compute the mean normal of every "s" points
for j in range(s,len(surface_normals),s):
    mean_points.append(np.array([np.mean(startpoint[j-s:j][:,0]), np.mean(startpoint[j-s:j][:,1]), np.mean(startpoint[j-s:j][:,2])]))
    n = np.array([np.mean(surface_normals[j-s:j][:,0]),np.mean(surface_normals[j-s:j][:,1]),np.mean(surface_normals[j-s:j][:,2])])
    n /= np.linalg.norm(n)
    mean_normals.append(n)
mean_points = np.asarray(mean_points)
mean_normals = np.asarray(mean_normals)

#find the inner product between the normals to identify a point of change, where a new plane should be defined.
i = 0
change_points = []
for k in range(0,len(mean_normals)-1):
    DOT = np.linalg.norm(np.rad2deg(np.arccos(np.dot(mean_normals[k],mean_normals[k+1])/(np.linalg.norm(mean_normals[k])*np.linalg.norm(mean_normals[k+1])))))
    if 30 < np.abs(DOT) < 90:
        change_points.append(mean_points[i+1])
    i += 1
change_points = np.asarray(change_points)
"""
fit1,c,err,inliers1,outliers1= ransacPlane(mean_points,2,0.2)
print(np.around(fit1,decimals = 4))
fit2,c,err,inliers2,outliers2 = ransacPlane(outliers1,0.5,0.1)
print(np.around(fit2,decimals = 4))
#print(np.abs(fit1[3]-fit2[3]))
#fit3,c,err,inliers3,outliers3 = ransacPlane(out2,0.5,0.1)
#print(np.around(fit3,decimals = 2))
#fit4,c,err,outliers4 = ransacPlane(outliers3,1.5,0.1)
#print(np.around(fit4,decimals = 2))
"""
x = points[:,0]
y = points[:,1]
z = points[:,2]
x_C = change_points[:,0]
y_C = change_points[:,1]
z_C = change_points[:,2]
plt.figure(1)
#plt.scatter(x,y)
ax = plt.subplot(111, projection ='3d')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_zlim(290,350)
#ax.scatter(x, y, z, s = 0.1, color ='b')
ax.quiver(mean_points[:,0],mean_points[:,1],mean_points[:,2],mean_normals[:,0],mean_normals[:,1],mean_normals[:,2],length = 3)
ax.scatter(mean_points[:,0],mean_points[:,1],mean_points[:,2],s = 0.1,color = "b")
#ax.scatter(x_C, y_C, z_C ,color ='r')
#ax.scatter(inliers2[:,0], inliers2[:,1], inliers2[:,2], color ='y')
#ax.scatter(out2[:,0], out2[:,1], out2[:,2], color ='r')
#plotPlane(fit1,ax,"g",0.3)
#plotPlane(fit2,ax,"y",0.3)
#plotPlane(fit3,ax,"b",1)
#plotPlane(fit4,ax,"y",1)
plt.show()

n = np.array([-4.5e-03,-1.6e-03,-1])
p_0 = np.array([0,0,301.10])
p = np.array([0,0,285.39])

print((np.dot(n,(p-p_0))/np.linalg.norm(n)))
