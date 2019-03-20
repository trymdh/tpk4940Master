from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import os
import glob
from AT_cx_functions import*
import random

#load all the calibration parameters
ret,K,tvecs,rMats,dist = loadCaliParam()
K_inv = np.linalg.inv(K)
os.chdir('C:/Users/Trym/OneDrive/Master/VS Code/laserimage') #home pc
#os.chdir('C:/Users/trymdh.WIN-NTNU-NO/OneDrive/Master/VS Code/laserimage') #work pc
#os.chdir('C:/Users/TrymAsus/OneDrive/Master/VS Code/laserimage')#LAPTOP

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
ext_points = ext_points[::100]

#RANSAC---------------------------------------------------------------
def estimatePlane(pointData,len_data):
    P_rand = []
    rand_index = random.sample(len_data,3)
    for i in rand_index:
        P_rand.append(pointData[i])
    P_rand = np.asarray(P_rand)

    p_1 = P_rand[0]
    p_2 = P_rand[1]
    p_3 = P_rand[2]

    #compute two vectors in the plane
    v1 = p_1 - p_3
    v2 = p_2 - p_3

    #The plane normal , n = [a,b,c]
    n = np.cross(v1,v2)
    
    #Plane, pI = [n,d] = [a,b,c,d]
    a,b,c = n
    d = -np.dot(p_3,np.cross(p_1,p_2))
    pI = np.array([a,b,c,d])
    
    #p_h = [x,y,z,1]
    p_1_h = np.append(p_1,1)
    p_2_h = np.append(p_2,1)
    p_3_h = np.append(p_3,1)

    #Criteria for a point to be in the plane is that the equation x_h . Pi = 0 is satisfied.
    c1 = np.dot(p_1_h,pI)
    c2 = np.dot(p_2_h,pI)
    c3 = np.dot(p_3_h,pI)
    
    conditions = [np.around(c1,decimals=6) == 0,
    np.around(c3,decimals=6) == 0,
    np.around(c3,decimals=6) == 0]

    if all(conditions):
        return pI, c1
    else:
        return None

def ransacPlane(pointData,goal_inliers,k_max,stop_at_goal = True):
    len_data = list(range(0,len(pointData)))
    best_fit = np.zeros(4)
    best_inlier = 0
    cnt_inliers = 0
    k = 0
    while best_inlier < goal_inliers:
        pI,thresh_inlier = estimatePlane(pointData,len_data)   
        for point in pointData:
            point = np.append(point,1)
            error = np.dot(point,pI)
            if error < thresh_inlier:
                cnt_inliers = cnt_inliers + 1

        if cnt_inliers > goal_inliers and stop_at_goal:
            best_inlier = cnt_inliers
            best_fit[0], best_fit[1], best_fit[2], best_fit[3] = pI/pI[2]

            print("Iteration Number {0}".format(k))
            print("Number of inliers is {0}".format(best_inlier))
            print("The plane equation is {0}x + {1}y + {2}z = {3}".format(best_fit[0],best_fit[1],best_fit[2],best_fit[3]))
        k = k + 1
    return best_fit

best_fit = ransacPlane(ext_points,1000,100)

a,b,c,d = best_fit
#plot raw data
x = ext_points[:,0]
y = ext_points[:,1]
z = ext_points[:,2]

plt.figure(1)
ax = plt.subplot(111, projection ='3d')
ax.scatter(x, y, z, color ='b')

#plot planes
xlim = ax.get_xlim()
ylim = ax.get_ylim()
X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),
                    np.arange(ylim[0], ylim[1]))

Z = (- a * X - b * Y - d)/ c

ax.plot_wireframe(X,Y,Z,alpha = 0.5, color = 'r')

ax.set_zlim(1200,0)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()
