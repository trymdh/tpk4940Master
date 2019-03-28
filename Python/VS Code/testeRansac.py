from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import os
import glob
from AT_cx_functions import*
from Robotfun import*

#load all the calibration parameters
ret,K,tvecs,rMats,dist = loadCaliParam()
K_inv = np.linalg.inv(K)
#os.chdir('C:/Users/Trym/OneDrive/Master/VS Code/laserimage') #home pc
os.chdir('C:/Users/trymdh.WIN-NTNU-NO/OneDrive/Master/VS Code/laserimage') #work pc
#os.chdir('C:/Users/TrymAsus/OneDrive/Master/VS Code/laserimage')#LAPTOP

laser_npy = os.getcwd().replace("\\","/") + "/*.npy"
laser_npy = glob.glob(laser_npy)
ext_points = extractPoints(laser_npy,rMats,tvecs,K_inv)

#ext_points = ext_points[2048:]
ext_points = ext_points[::50]

bestFit,c,bestError = ransacPlane(ext_points)

x = ext_points[:,0]
y = ext_points[:,1]
z = ext_points[:,2]

plt.figure(1)
ax = plt.subplot(111, projection ='3d')
ax.scatter(x, y, z,color ='b')
ax.scatter(0,0,0,'x',color = 'r')
ax.scatter(c[0],c[1],c[2],'o',s = 50, color = 'r' )

X,Y,Z = getPlaneData(bestFit,ax,ls = True)

ax.plot_wireframe(X,Y,Z, color='r',alpha = 0.2)
ax.set_zlim(np.max(z),0)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()
