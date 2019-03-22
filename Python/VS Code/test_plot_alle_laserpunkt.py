from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import os
import glob
from AT_cx_functions import*

#load all the calibration parameters
ret,K,tvecs,rMats,dist = loadCaliParam()
K_inv = np.linalg.inv(K)
os.chdir('C:/Users/Trym/OneDrive/tpk4940Master/Python/VS Code/laserimage') #home pc
#os.chdir('C:/Users/trymdh.WIN-NTNU-NO/OneDrive/tpk4940Master/Python/VS Code/laserimage') #work pc
#os.chdir('C:/Users/TrymAsus//OneDrive/tpk4940Master/Python/VS Code/laserimage')#LAPTOP

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
#ext_points = ext_points[::100]

x = ext_points[:,0]
y = ext_points[:,1]
z = ext_points[:,2]

#Centroid of the pointcloud (average point)
C = []
x_av = np.average(x)
y_av = np.average(y)
z_av = np.average(z)
C.append(x_av)
C.append(y_av)
C.append(z_av)



"""
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x,y,z, c='r', marker='o')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_zlim(1200,0)
plt.show()
"""
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

#Least Squares Method
#In many cases LSM is a good enough method for estimation, but it is sensitive to outliers which may occur when using noisy point data.
"""
Code below is based on the user "BEN" from 
https://stackoverflow.com/questions/1400213/3d-least-squares-plane
"""
#----------------------------------------------------------------------------------------------------------------------------
# A * x_fit = b, where A = [[x1 y1 1],  and x_fit = [a b c], and B = [z1, z2, ... , zn]
#                    [x2 y2 1],
#                      ...,
#                    [xn yn 1]]
# Solve for x_fit => plane = ax + by + c = z
tmp_A = []
tmp_b = []
for i in range(len(x)):
    tmp_A.append([x[i], y[i], 1])
    tmp_b.append(z[i])
B = np.matrix(tmp_b).T
A = np.matrix(tmp_A)

x_fit = (A.T * A).I * A.T * B # x_fit = inverse(A' * A) * (A' * B)
errors = B - A * x_fit
residual = np.linalg.norm(errors)/len(ext_points)

print ("LS solution:")
print ("%f x + %f y + %f = z" % (x_fit[0], x_fit[1], x_fit[2]))
print ("residual:")
print(residual)

#lin_fit,res,rank,s = np.linalg.lstsq(A,B)
#print(lin_fit,res,rank,s)
#RANSAC
#-------------------------------------------------------------------------------------------------------------------------------







#------------------------------------------------------------------------------------------------------------------------------
ext_points = ext_points[::100]

x = ext_points[:,0]
y = ext_points[:,1]
z = ext_points[:,2]

# plot raw data
plt.figure(1)
ax = plt.subplot(111, projection ='3d')
ax.scatter(x, y, z, color ='b')

#plot centroid
ax.scatter(C[0],C[1],C[2], s = 2000 ,color = 'y', marker = "x")

#plot planes
xlim = ax.get_xlim()
ylim = ax.get_ylim()
X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),
                  np.arange(ylim[0], ylim[1]))
Z = np.zeros(X.shape)
"""
#SVD-plane
for r in range(X.shape[0]):
    for c in range(X.shape[1]):
        Z[r,c] = pI[0]/pI[2] * X[r,c] + pI[1]/pI[2] * Y[r,c] + pI[3]/pI[2]
ax.plot_wireframe(X,Y,Z, color='r')
"""

#LS-plane
for r in range(X.shape[0]):
    for c in range(X.shape[1]):
        Z[r,c] = x_fit[0] * X[r,c] + x_fit[1] * Y[r,c] + x_fit[2]
ax.plot_wireframe(X,Y,Z, color='g')

ax.set_zlim(1200,0)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()
