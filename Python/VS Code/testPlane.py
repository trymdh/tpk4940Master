from AT_cx_functions import*
from matplotlib import pyplot as plt
import numpy as np
import glob
import os
from mpl_toolkits.mplot3d import Axes3D

#load all the calibration parameters
ret,K,tvecs,rMats,dist = loadCaliParam()
K_inv = np.linalg.inv(K)
#os.chdir('C:/Users/Trym/OneDrive/Master/VS Code/laserimage') #home pc
os.chdir('C:/Users/trymdh.WIN-NTNU-NO/OneDrive/Master/VS Code/laserimage') #work pc
#os.chdir('C:/Users/TrymAsus/OneDrive/Master/VS Code/laserimage')#LAPTOP

laser_npy = os.getcwd().replace("\\","/") + "/*.npy"
laser_npy = glob.glob(laser_npy)
ext_points = extractPoints(laser_npy,rMats,tvecs,K_inv)
#maybeIndex = np.random.choice(ext_points.shape[0],3,replace = False)
#G = ext_points
#G = ext_points[maybeIndex,:]
G = np.array([[0,0,0],[0,1,0],[1,1,0]])

x = G[:,0]
y = G[:,1]
z = G[:,2]

#Centroid
C = getCentroid3D(G)
C_h = np.append(C,1)



#generated pointCloud containing outliers
X = []
for i in range(0,1000):
    X.append(np.random.rand(1,3))

X = np.array(X).reshape(-1,3)

#maybe model made from maybe inliers
G_h = homogenify(G)
pI = svd_AxB(G_h)

xxx = [0,0,-5,1]
xxx = np.array(xxx)
pI = np.array([0,0,-1,3])
d = (np.dot((point - maybeModel[3]*maybeModel[0:3]),maybeModel[0:3]))
print(d)

#print(pI/pI[2])

error = 0
error_list = []
for point in X:
    point_h = np.append(point,1)
    error_list.append(np.linalg.norm(np.dot(pI,point_h.T)))

median_error = np.median(error_list)
error_std = np.std(error_list)

inliers= []
outliers = []
cnt_in = 0
cnt_out = 0

i = 0
for error in error_list:
    if np.abs(error) < 1.5*error_std:
        inliers.append(X[i])
        cnt_in += 1
    else:
        outliers.append(X[i])
        cnt_out += 1
    i += 1

inliers = np.array(inliers)
xx_i = inliers[:,0]
yy_i = inliers[:,1]
zz_i = inliers[:,2]

in_plane = svd_AxB(homogenify(inliers))
print(in_plane/in_plane[2])

outliers = np.array(outliers)
xx_o = outliers[:,0]
yy_o = outliers[:,1]
zz_o = outliers[:,2]

plt.figure(1)
ax = plt.subplot(111, projection ='3d')

ax.scatter(x, y, z, color ='b')
ax.scatter(xx_o,yy_o,zz_o, s = 10 ,color = 'r', marker = "x")
ax.scatter(xx_i,yy_i,zz_i, s = 10 ,color = 'g', marker = "o")
ax.scatter(C[0],C[1],C[2], s = 10 ,color = 'y', marker = "x")

#plot planes
X,Y,Z = getPlaneData(pI,ax,rnsc = True)

ax.plot_wireframe(X,Y,Z, color='r',alpha=0.5)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()

