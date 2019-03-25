from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import os
import glob
from AT_cx_functions import*

#load all the calibration parameters
ret,K,tvecs,rMats,dist = loadCaliParam()
K_inv = np.linalg.inv(K)
#os.chdir('C:/Users/Trym/OneDrive/Master/VS Code/laserimage') #home pc
os.chdir('C:/Users/trymdh.WIN-NTNU-NO/OneDrive/Master/VS Code/laserimage') #work pc
#os.chdir('C:/Users/TrymAsus/OneDrive/Master/VS Code/laserimage')#LAPTOP

laser_npy = os.getcwd().replace("\\","/") + "/*.npy"
laser_npy = glob.glob(laser_npy)
ext_points = np.array([])
j = 0
for i in range(1,len(laser_npy) + 1):
    RotM = rMats[j]
    tVec = tvecs[j]
    T = np.eye(4)
    T[0:3,0:3] = RotM
    T[0:3,3] = tVec
    n = RotM[2,:]
    p_0 = tVec
    l_0 = np.array([0,0,0])
    filename = 'pixcoord_' + str(i) + '.npy'
    pix_coord = np.load(filename)
    
    for coord in pix_coord:
        if coord[1] != 0:
            coord = np.append(coord,1)
            img_coord = K_inv@coord
            norm_img_coord = img_coord/img_coord[2]
            l = norm_img_coord
            num = np.dot((p_0 - l_0), n)
            deno = np.dot(l,n)
            d = num/deno
            fullcoord = np.array([norm_img_coord * d]) + l_0
            ext_points = np.append(ext_points,fullcoord)
    j = j + 1

ext_points = np.reshape(ext_points,(-1,3))
ext_points = ext_points[2048:]
ext_points = ext_points[::50]
#RANSAC---------------------------------------------------------------
fit = ransacPlane(ext_points,0.7*len(ext_points),1000,0.1,True)

#Least square fit
#fit,res =  lsPlane(ext_points,True)


#a,b,c,d = best_fit
#plot raw data
x = ext_points[:,0]
y = ext_points[:,1]
z = ext_points[:,2]

plt.figure(1)
ax = plt.subplot(111, projection ='3d')
ax.scatter(x, y, z, color ='b')

#plot planes
X,Y,Z = getPlaneData(fit,ax)
ax.plot_wireframe(X,Y,Z, color='r')
ax.set_zlim(1200,0)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()
