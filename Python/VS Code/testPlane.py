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
G = np.array([[1,0,0],[0,1,1],[1,1,0]])

x = G[:,0]
y = G[:,1]
z = G[:,2]

C = getCentroid3D(G)
C_h = np.append(C,1)

X = G[0]-np.array([0.2,0,1])
X_h = np.append(X,1)

#maybe inliers
P_rand = G[random.sample(list(range(0,len(G))),3),:]
#maybe model made from maybe inliers
pI = svd_AxB(G)
print(pI)
plt.figure(1)
ax = plt.subplot(111, projection ='3d')
print(np.linalg.norm(np.dot(pI,X_h.T)))
print(np.linalg.norm(np.dot(pI,C_h.T)))
print(np.linalg.norm(np.dot(pI,np.append(G[0],1)).T))


ax.scatter(x, y, z, color ='b')
ax.scatter(X[0],X[1],X[2], s = 10 ,color = 'r', marker = "x")
ax.scatter(C[0],C[1],C[2], s = 10 ,color = 'y', marker = "x")

#plot planes
X,Y,Z = getPlaneData(pI,ax,rnsc = True)

ax.plot_wireframe(X,Y,Z, color='r',alpha=0.5)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()

