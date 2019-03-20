from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D



x1 = np.array([0,0,1,1])
x2 = np.array([1,1,1,1])
x3 = np.array([1,2,1,1])

G = np.array([x1,
x2,
x3])
x = G[:,0]
y = G[:,1]
z = G[:,2]

u,s,vh = np.linalg.svd(G)
v = vh.T
fit = v[:,3]
print(G@fit)
print ("SVD solution:")
print ("%f x + %f y + %f = z" % (fit[0], fit[1], fit[2]))

tmp_A = []
tmp_b = []
for i in range(len(x)):
    tmp_A.append([x[i], y[i], 1])
    tmp_b.append(z[i])
b = np.matrix(tmp_b).T
A = np.matrix(tmp_A)
fit = (A.T * A).I * A.T * b

errors = b - A * fit
residual = np.linalg.norm(errors)

print ("LS solution:")
print ("%f x + %f y + %f = z" % (fit[0], fit[1], fit[2]))

# plot raw data
plt.figure()
ax = plt.subplot(111, projection ='3d')
ax.scatter(x, y, z, color ='b')
ax.set_zlim(1200,0)

# plot plane
xlim = ax.get_xlim()
ylim = ax.get_ylim()
X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),
                  np.arange(ylim[0], ylim[1]))
Z = np.zeros(X.shape)
for r in range(X.shape[0]):
    for c in range(X.shape[1]):
        Z[r,c] = fit[0] * X[r,c] + fit[1] * Y[r,c] + fit[2]
ax.plot_wireframe(X,Y,Z, color='k')

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()
