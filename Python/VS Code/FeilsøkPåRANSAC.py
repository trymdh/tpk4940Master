from AT_cx_functions import*
from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


N_POINTS = 120
TARGET_X_SLOPE = 0
TARGET_y_SLOPE = 3
TARGET_OFFSET  = 5
EXTENTS = 5
NOISE = 10

# create random data
xs = [np.random.uniform(2*EXTENTS)-EXTENTS for i in range(N_POINTS)]
ys = [np.random.uniform(2*EXTENTS)-EXTENTS for i in range(N_POINTS)]
zs = []
for i in range(N_POINTS):
    zs.append(xs[i]*TARGET_X_SLOPE + \
              ys[i]*TARGET_y_SLOPE + \
              TARGET_OFFSET + np.random.normal(scale=NOISE))

# plot raw data
plt.figure()
ax = plt.subplot(111, projection='3d')
ax.scatter(xs, ys, zs, color='b')
#plt.show()

G = np.zeros((N_POINTS,3))
G[:,0] = xs
G[:,1] = ys
G[:,2] = zs

n,c,d = svd_AxB(homogenify(G))
pI = np.append(n,d)

n_r,c_r,err_r = ransacPlane(G)
print(pI[0:3]/np.linalg.norm(pI[0:3]),d)
print(n_r[0:3]/np.linalg.norm(n_r[0:3]),n_r[3])

# plot plane
xlim = ax.get_xlim()
ylim = ax.get_ylim()
X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),
                  np.arange(ylim[0], ylim[1]))
Z = np.zeros(X.shape)
Z_ransac = np.zeros(X.shape)
for r in range(X.shape[0]):
    for c in range(X.shape[1]):
        Z[r,c] = -(n[0] * X[r,c] + n[1] * Y[r,c])*(1./n[2]) - d
        Z_ransac[r,c] = -(n_r[0] * X[r,c] + n_r[1] * Y[r,c])*(1./n_r[2]) - n_r[3]

ax.plot_wireframe(X,Y,Z, color='r')
ax.plot_wireframe(X,Y,Z_ransac, color='g')

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()