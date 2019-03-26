from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import os
import glob
from AT_cx_functions import*

#load all the calibration parameters
ret,K,tvecs,rMats,dist = loadCaliParam()
K_inv = np.linalg.inv(K)
os.chdir('C:/Users/Trym/OneDrive/Master/VS Code/laserimage') #home pc
#os.chdir('C:/Users/trymdh.WIN-NTNU-NO/OneDrive/Master/VS Code/laserimage') #work pc
#os.chdir('C:/Users/TrymAsus/OneDrive/Master/VS Code/laserimage')#LAPTOP

laser_npy = os.getcwd().replace("\\","/") + "/*.npy"
laser_npy = glob.glob(laser_npy)
ext_points = extractPoints(laser_npy,rMats,tvecs,K_inv)

ext_points = np.reshape(ext_points,(-1,3))
ext_points = ext_points[2048:]
ext_points = ext_points[::50]
#RANSAC---------------------------------------------------------------
ransac_fit,c,err = ransacPlane(ext_points)

#Least square fit
ls_fit,res =  lsPlane(ext_points,True)
print(ls_fit)

#plot raw data
x = ext_points[:,0]
y = ext_points[:,1]
z = ext_points[:,2]

plt.figure(1)
ax = plt.subplot(111, projection ='3d')
ax.scatter(x, y, z, color ='b')

#plot planes
#LS:
X,Y,Z = getPlaneData(ls_fit,ax,ls = True)
ax.plot_wireframe(X,Y,Z, color='r')

#RANSAC:
X,Y,Z = getPlaneData(ransac_fit,ax,rnsc = True)
ax.plot_wireframe(X,Y,Z, color='g')

ax.set_zlim(1200,0)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()
