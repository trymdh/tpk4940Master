from AT_cx_functions import*
from matplotlib import pyplot as plt
import numpy as np

from mpl_toolkits.mplot3d import Axes3D

G = np.array([[1,0,2],[0,0,1],[1,1,0]])
x = G[:,0]
y = G[:,1]
z = G[:,2]

C = getCentroid3D(x,y,z)

#maybe inliers
P_rand = G[random.sample(list(range(0,len(G))),3),:]
print(P_rand)
#maybe model made from maybe inliers
pI = estimatePlane(P_rand)


plt.figure(1)
ax = plt.subplot(111, projection ='3d')

ax.scatter(x, y, z, color ='b')
ax.scatter(C[0],C[1],C[2], s = 10 ,color = 'y', marker = "x")

#plot planes
X,Y,Z = getPlaneData(pI,ax)

ax.plot_wireframe(X,Y,Z, color='r')

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()


a = [1,2,3]

xx,zz = lsPlane(G)
print(xx,zz)