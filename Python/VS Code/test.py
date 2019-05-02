import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import os
import glob
from AT_cx_functions import*

#Import values from calibration process
ret,K,tvecs,rMats,dist = loadCaliParam()
K_inv = np.linalg.inv(K)
os.chdir('C:/Users/trymdh.WIN-NTNU-NO/OneDrive/tpk4940Master/Python/VS Code/laserimage')
laser_npy = os.getcwd().replace("\\","/") + "/*.npy"
laser_npy = glob.glob(laser_npy)
number_of_laserfiles = len(laser_npy)

ext_points = extractPoints(laser_npy,rMats,tvecs,K,dist)
ite = 0
n = 100
ransac_error_list = []
ls_error_list = []
while ite < n:
    #Adding noise to the dataset
    ext_points1 = list(ext_points)
    for i in range(0,1000):
        ext_points1.append([np.random.randint(0,1000),np.random.randint(0,1000),np.random.randint(0,1000)])
    ext_points1 = np.asarray(ext_points1)

    #least square fit
    ls_fit = lsPlane(ext_points1)
    ls_plane,ls_plane_s = planeify(ls_fit)
    error_LS = error_checker(-ls_plane,ext_points)

    #Ransac Plane
    ransac_fit,c,ransac_error = ransacPlane(ext_points1)
    
    ransac_error_list.append(ransac_error)
    ls_error_list.append(error_LS)
    ite = ite + 1
    print(ite)

i = list(range(0,n))
plt.plot(i,ransac_error_list,'r')
plt.plot(i,ls_error_list,'b')
plt.xlabel("iterations")
plt.ylabel("Mean Error")
plt.show()


"""
#plot the points and the planes
ext_points = ext_points[::100]
x = ext_points[:,0]
y = ext_points[:,1]
z = ext_points[:,2]

plt.figure(1)
ax = plt.subplot(111, projection ='3d')
ax.scatter(x, y, z, color ='b')
X_r,Y_r,Z_r = getPlaneData(ransac_fit,ax)
ax.plot_wireframe(X_r,Y_r,Z_r, color='g')
X_ls,Y_ls,Z_ls = getPlaneData(ls_plane,ax)
ax.plot_wireframe(X_ls,Y_ls,Z_ls, color='r')
plt.show()
"""