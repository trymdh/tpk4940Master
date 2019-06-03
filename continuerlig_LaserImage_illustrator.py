import os
import time
import glob
import msvcrt
import numpy as np
import cv2 as cv2
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from UtilityFunctions import getUname, sortList
from loadCalibration import loadCaliParam
from CameraFunctions import getDevice, snap, imageConfig, cogConfig, pixCoordify

def triang(pix):
    uname = getUname()
    t = time.time()
    ret,K,tvecs,rMats,dist= loadCaliParam(uname)
    laser = np.load("C:/Users/" + str(uname) + "/OneDrive/tpk4940Master/BestRansacPlane.npy")

    #1 Undistort the pixels
    pix = pix.reshape(-2, 1, 2)
    undistorted_pixels = cv2.undistortPoints(pix, K, dist).reshape(-1,2)
    #2 Triangulate
    ext_points = np.array([])

    #Define lasernormal and laserpoint
    ln = laser[0:3]
    ln[0:2] = ln[0:2]
    z_c = np.array([0,0,1])
    theta = np.pi - np.arccos(np.dot(ln,z_c)/(np.linalg.norm(ln)*np.linalg.norm(z_c)))
    lp = laser[3:6]
    l_0 = np.array([0,0,0])

    for coord in undistorted_pixels:
        coord = np.append(coord,1)
        l = coord/np.linalg.norm(coord)
        d = np.dot((lp - l_0), ln)/np.dot(l,ln)
        fullcoord = np.array([l * d]) + l_0
        ext_points = np.append(ext_points,fullcoord) 
    elapsed = time.time() - t

    return np.reshape(ext_points,(-1,3))

def in1d_dot_approach(A,B):
    cumdims = (np.maximum(A.max(),B.max())+1)**np.arange(B.shape[1])
    return A[~np.in1d(A.dot(cumdims),B.dot(cumdims))]

#Find and configure the device
hDev = getDevice()
cogConfig(hDev)
uname = getUname()
dead_array = np.zeros((2048,2))

for i in range(0,2048):
    dead_array[i,0] = i

dead_points = triang(dead_array)
indx = 0
loop_exit = True
while loop_exit:
    #Snap an image of the scene in COG mode.
    laserlinepixels = snap(hDev)
    #reshape the COG-image from (2048,) to (2048,2)
    laserpixel = pixCoordify(laserlinepixels.ravel() / 64 , 2048)
    points = triang(laserpixel)
    #Removes non registered pixels from the triangulated points, 
    # NB! clean_points are the points we want to use to building a pointcloud
    clean_points = in1d_dot_approach(points,dead_points)
    """
    X = np.load("X.npy")
    base_tool = np.load("T.npy") # this should update evry time the robot moves
    Y = base_tool@X
    points_EE = []
    for point in clean_points:
        points_EE.append(np.dot(Y,np.append(point,1).reshape(4,1)))
    clean_points = np.asarray(points_EE)
    """
    clean_points = clean_points[600:1400] # don care about the edges of the image...800 points

    if msvcrt.kbhit():
        key = str(msvcrt.getch()).replace("b'","").replace("'","")
        if key == 'e':
            loop_exit = False
            print("Loop exited")
        elif key == "s":
            os.chdir("C:/Users/"+ str(uname) + "/OneDrive/tpk4940Master/snaps1")
            laserPixelList = sortList(glob.glob(os.getcwd().replace("\\","/") + "/*.npy"))
            indx = len(laserPixelList) + 1
            np.save("snap" + str(indx) + ".npy",clean_points)
    plt.clf()
    plt.ion()
    fig = plt.figure(1)
    ax = plt.axes(projection='3d')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    X = clean_points[:,0]
    Y = clean_points[:,1]
    Z = clean_points[:,2]
    #ax.set_aspect("equal")
    ax.set_zlim3d(np.mean(Z)-50,np.mean(Z)+50)
    ax.set_ylim3d(np.mean(Y)-50,np.mean(Y)+50)
    ax.set_xlim3d(np.mean(X)-50,np.mean(X)+50)
    ax.scatter3D(X,Y,Z,s=0.1)
    plt.pause(1/60)
