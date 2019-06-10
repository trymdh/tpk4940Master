import os
import glob
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from loadCalibration import loadCaliParam
from PointArrayFunctions import LaserPointCloud, homogenify
from UtilityFunctions import getUname, sortList
from ErrorFunctions import getError, error_checker
from RansacFunctions import ransacPlane, ransacXn, countInliers, getCentroid3D
from PlaneFunctions import plotPlane, getPlaneData, getCentroid3D, lsPlane, planeify, svd_AxB

"""
This script estimates the laser plane in camera coordinates based on the RANSAC algorithm.
Input to the RANSAC function is a 3D pointcloud of the laserpoints as seen from the camera
frame. 

The laser pointcloud is calculated based on calibration parameters from the 
intrinsic camera calibration done in MATLAB. The parameters are exported to Python via loadCaliParam()
and input in LaserPointsCloud() which triangulates the laser points and returns a pointcloud
of all the triangulated laser points in camera coordinates.
"""

uname = getUname()
wd = os.getcwd()

#load laser images
os.chdir("C:/Users/" + str(uname) + "/OneDrive/tpk4940Master/Camera calibration May/LaserImages")
laser_fnames_list = sortList(glob.glob(os.getcwd().replace("\\","/") + "/*.npy"))
laserImages = []
for fname in laser_fnames_list:
    laserImages.append(np.load(fname))
laserImgs = np.asarray(laserImages)

#triangulate laser points from laser images
pointCloud = LaserPointCloud(uname,laserImgs)

#Estimate LS Plane 
ls_fit = lsPlane(pointCloud)
ls_plane,ls_plane_s = planeify(ls_fit)
error_LS = error_checker(-ls_plane,pointCloud)

#Estimate ransac plane
bestFit,c,bestErr = ransacXn(uname,pointCloud,10)

#plot laser points
pointCloud = pointCloud[::50]
x = pointCloud[:,0]
y = pointCloud[:,1]
z = pointCloud[:,2]

#plot planes
plt.figure(1)
ax = plt.subplot(111, projection ='3d')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.scatter(x, y, z, color ='b')
plotPlane(bestFit,ax,"g",1) #ransac fit
plotPlane(ls_plane,ax,"r",0.3) #least squares fit
plt.show()

print("Ransac plane: {0} \nError: {1}".format(bestFit,bestErr))
print("LS plane: {0} \nError: {1}".format(-ls_plane,error_LS))
