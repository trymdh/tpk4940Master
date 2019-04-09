from AT_cx_functions import*
from matplotlib import pyplot as plt
import numpy as np
import glob
import os
from mpl_toolkits.mplot3d import Axes3D

#G = ext_points
#G = ext_points[maybeIndex,:]
G = np.array([[1,0,0],[0,1,1],[1,1,0]])

x = G[:,0]
y = G[:,1]
z = G[:,2]

#Centroid
C = getCentroid3D(G)
C_h = np.append(C,1)

#generated pointCloud containing outliers and inliers
X = []
for i in range(0,1000):
    X.append(np.random.rand(1,3))

X = np.array(X).reshape(-1,3)

#maybe model made from maybe inliers
G_h = homogenify(G)
pI,c = svd_AxB(G_h)

error_vec,median_error,error_std = getError(X,pI,c)
print(median_error,error_std)

inliers= []
outliers = []
cnt_in = 0
cnt_out = 0

i = 0
for error in error_vec:
    if np.abs(error) < median_error- 0.5*error_std:
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
X,Y,Z = getPlaneData(pI,ax)

ax.plot_wireframe(X,Y,Z, color='b',alpha=1)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()

