from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

x1 = []
y1= []
z1 = []
for i in range(100):
    x1.append([i,1,0])
    y1.append([1,i,0])
    z1.append([1,0,i])



plt.figure(1)
ax = plt.subplot(111, projection ='3d')
ax.scatter(x1, y1, z1, color ='b')
plt.show()