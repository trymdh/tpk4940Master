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
ext_points = ext_points[::100]

bestFit,c,bestError = ransacPlane(ext_points)
print(bestFit,bestError)

x = ext_points[:,0]
y = ext_points[:,1]
z = ext_points[:,2]


normal = bestFit[0:3]
d = -np.sum(c*normal)

plt.figure(1)
ax = plt.subplot(111, projection ='3d')

ax.scatter(x, y, z, color ='b')

X,Y,Z = getPlaneData(bestFit,ax,rnsc = True)

#Z = (-normal[0]*X - normal[1]*Y - d)*1./normal[2]

ax.plot_wireframe(X,Y,Z, color='r')
ax.set_zlim(np.max(z),0)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()
