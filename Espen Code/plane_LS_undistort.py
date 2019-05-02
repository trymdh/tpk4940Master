from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import os
import glob
import cv2 as cv2
from loadCali import loadCaliParam
from planeify import planeify

ret,rets,K,tvecs,rMats,dist,rotVecError,transVecError = loadCaliParam()
print(K)
K_inv = np.linalg.inv(K)
os.chdir('C:/Users/espen/OneDrive/Master/Laserplankalibrering/Bilder_og_koordinater/LaserAndNpys')
laser_npy = os.getcwd().replace("\\","/") + "/*.npy"
laser_npy = glob.glob(laser_npy)
number_of_laserfiles = len(laser_npy)
#print(number_of_laserfiles)
ext_points = np.array([])
j = 0
#print(tvecs.shape)
for i in range(1,len(laser_npy)+1):

    RotM = rMats[j]
    print(i)
    tVec = tvecs[j]
    T = np.eye(4)
    T[0:3,0:3] = RotM
    T[0:3,3] = tVec
    n = RotM[2,:]
    p_0 = tVec
    l_0 = np.array([0,0,0])
    filename = 'pixcoord_' + str(i) + '.npy'
    pix_coord = np.load(filename)

    k = 0


    pure_coords = np.array([])
    for reading in pix_coord:
        if reading[1] != 0 and 650<=k<=1700 : #Remove pixels beond 1700 due to interference
                pix = [reading[0],reading[1]]
                pure_coords = np.append(pure_coords,pix)

        k += 1                     


    pix_coord = pure_coords.reshape(-2, 1, 2)

    undistorted_point = cv2.undistortPoints(pix_coord, K, dist).reshape(-1,2)
 
    for coord in undistorted_point:
        if coord[1] > -0.5:
                #print(coord)
                coord = np.append(coord,1)
                l = coord/np.linalg.norm(coord)
                num = np.dot((p_0 - l_0), n)
                deno = np.dot(l,n)
                d = num/deno
                fullcoord = np.array([l * d]) + l_0
                ext_points = np.append(ext_points,fullcoord) 
        else:
                print("coord is zero-equivalent")

    j = j + 1


ext_points = np.reshape(ext_points,(-1,3))

"""
ext_points = ext_points[2048:]
ext_points = ext_points[::100]
"""
x = ext_points[:,0]
y = ext_points[:,1]
z = ext_points[:,2]


"""
Code below is based on the user "BEN" from 
https://stackoverflow.com/questions/1400213/3d-least-squares-plane
"""
# plot raw data
plt.figure()
ax = plt.subplot(111, projection='3d')
ax.scatter(x, y, z, color='b')

tmp_A = []
tmp_b = []
for i in range(len(x)):
    tmp_A.append([x[i], y[i], 1])
    tmp_b.append(z[i])
b = np.matrix(tmp_b).T
A = np.matrix(tmp_A)
fit = (A.T * A).I * A.T * b
fit_QP = np.array([0.032640206253893515, 2.631808058207668, 600.0306267043595]).reshape(3,1)



errors_QP = b - A * fit_QP
errors = b - A * fit

residual = np.linalg.norm(errors)
residual_QP = np.linalg.norm(errors_QP)

print ("solution:")
print ("%f x + %f y + %f = z" % (fit[0], fit[1], fit[2]))
print ("errors:")
print (errors)
print ("errors QP:")
print (errors_QP)
print ("residual:")
print (residual)
print ("residual QP:")
print (residual_QP)


plane_vec = [float(fit[0]), float(fit[1]), -1]
norm_plane_vec = plane_vec 
print(norm_plane_vec)
point_on_plane = [0,0,float(fit[2])]
plane_representation = [norm_plane_vec[0],norm_plane_vec[1],norm_plane_vec[2],point_on_plane[0],point_on_plane[1],point_on_plane[2]]
os.chdir('C:/Users/espen/OneDrive/Master/Laserplankalibrering/Bilder_og_koordinater')
print(plane_representation)
np.save('laserplane_LS2.npy',plane_representation)
"""
Faktisk Error:


"""
print(len(errors))
# plot plane
xlim = ax.get_xlim()
ylim = ax.get_ylim()
X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),
                  np.arange(ylim[0], ylim[1]))
Z = np.zeros(X.shape)
for r in range(X.shape[0]):
    for c in range(X.shape[1]):
        Z[r,c] = fit[0] * X[r,c] + fit[1] * Y[r,c] + fit[2]
#ax.plot_wireframe(X,Y,Z, color='k')

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()
print(fit)



fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x,y,z, c='r', marker='o')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_ylim(-150,200)
ax.set_zlim(1200,0)
plt.show()

