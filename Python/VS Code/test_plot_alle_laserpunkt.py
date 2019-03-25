from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import os
import glob
from AT_cx_functions import*

#load all the calibration parameters
ret,K,tvecs,rMats,dist = loadCaliParam()
K_inv = np.linalg.inv(K)
#os.chdir('C:/Users/Trym/OneDrive/tpk4940Master/Python/VS Code/laserimage') #home pc
os.chdir('C:/Users/trymdh.WIN-NTNU-NO/OneDrive/tpk4940Master/Python/VS Code/laserimage') #work pc
#os.chdir('C:/Users/TrymAsus//OneDrive/tpk4940Master/Python/VS Code/laserimage')#LAPTOP

laser_npy = os.getcwd().replace("\\","/") + "/*.npy"
laser_npy = glob.glob(laser_npy)

ext_points = extractPoints(laser_npy,rMats,tvecs,K_inv)
ext_points = ext_points[2048:]
ext_points = ext_points[::100]

#Centroid of the pointcloud (average point)
C = getCentroid3D(ext_points)

#SVD Method
# lookup "best fit plane to 3D data: different results for two different methods"
#----------------------------------------------------------------------------------------------------------------------------
# General plane equation: Ax + By + Cz + D = 0
"""
G_tmp = []
for i in range(len(x)):
    G_tmp.append([x[i], y[i], -z[i], 1])
    G = np.matrix(G_tmp)

u, s, vh = np.linalg.svd(G, full_matrices=True)

v = vh.T
pI = v[:,3] # pI = [A B C D]', plane in Homogeneous coordinates

error = G@pI
residual = np.linalg.norm(error)
print ("SVD solution:")
#print ("%f x + %f y + %f z + %f = 0" % (pI[0], pI[1], pI[2], pI[3]))
print ("%f x + %f y + %f = z" % (pI[0]/pI[2], pI[1]/pI[2], pI[3]/pI[2]))
print ("residual:")
print (residual)
"""
#------------------------------------------------------------------------------------------------------------------------------


x = ext_points[:,0]
y = ext_points[:,1]
z = ext_points[:,2]

# plot raw data
plt.figure(1)
ax = plt.subplot(111, projection ='3d')
ax.scatter(x, y, z, color ='b')

#plot centroid
ax.scatter(C[0],C[1],C[2], s = 2000 ,color = 'y', marker = "x")

ax.set_zlim(1200,0)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()
