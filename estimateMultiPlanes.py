import os
import numpy as np 
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from UtilityFunctions import getUname, getCurrentWD
from RansacFunctions import ransacPlane
from PlaneFunctions import plotPlane

uname = getUname()
wd = getCurrentWD()
laser3D = np.load("POOOOOOINTS.npy")[700:1200]
base_tool = np.load("T.npy")
base_tool[0:3,3] = base_tool[0:3,3]*1000
X = np.load("X.npy")
Y = base_tool@X

points_Y = []
for j in range(0,100):
    #simulating movement along X axes
    Y[:3,3][0] += 1
    #Y[:3,3][2] += 0.1
    for point in laser3D:
        points_Y.append(np.dot(Y,np.append(point,1)))
points_Y = np.asarray(points_Y)
points_Y = points_Y
print(points_Y.shape[1])

#"Unhomogenize" the pointcloud
points_Y = np.column_stack((np.column_stack((points_Y[:,0],points_Y[:,1])),points_Y[:,2]))

fit1,c,err,outliers = ransacPlane(points_Y)
fit2,c,err,newoutliers = ransacPlane(outliers)
fit3,c,err,newnewoutliers = ransacPlane(newoutliers)
print(np.around(fit1,decimals = 2))
print(np.around(fit2,decimals = 2))
print(np.around(fit3,decimals = 2))
outliers = outliers[::50]
x = outliers[:,0]
y = outliers[:,1]
z = outliers[:,2]
#x = points_Y[:,0]
#y = points_Y[:,1]
#z = points_Y[:,2]
plt.figure(1)
ax = plt.subplot(111, projection ='3d')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.scatter(x, y, z, color ='b')
plotPlane(fit1,ax,"g",1)
plotPlane(fit2,ax,"r",1)
plotPlane(fit3,ax,"b",1)
plt.show()