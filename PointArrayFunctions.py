import os
import cv2
import glob
import numpy as np
from loadCalibration import loadCaliParam

def cleanUpLaser(laserpixel):
    #sets points that are 0 to None, this way they are not plotted by matplotlib
    clean_laser = []
    for point in laserpixel:
        if point[1] == 0:
            point[1] = None
        clean_laser.append(point)
    return np.asarray(clean_laser)

def LaserPointCloud(uname,laserImgs):
    ret,K,t_vecs,R_mats,dist = loadCaliParam(uname)
    ext_points = []
    l_0 = np.array([0,0,0])
    j = 0
    cnt = 0
    for pix in laserImgs:
        pix = pix.reshape(-2,1,2)
        for pxls in pix:
            if pxls[0][1] == 0:
                cnt +=1
            #only calculate the "found" laser points, i.e point that dont have zero px_y coordinate.
            if pxls[0][1] != 0:
                pxls = pxls[0].reshape(-2,1,2)
                R = R_mats[j]
                t = t_vecs[j]
                T = np.eye(4)
                n = R[2,:]
                p_0 = t
                undist_pix = cv2.undistortPoints(pxls,K,dist).reshape(-1,2) #undistorted normalized image coordinate
                for coord in undist_pix:
                    #make the point homogeneous, i.e (x,y,1)
                    norm_img_coord = np.append(coord,1)
                    #triangulate the laserpoint in camera coordinates
                    l = norm_img_coord/np.linalg.norm(norm_img_coord)
                    d = np.dot((p_0 - l_0),n)/np.dot(l,n)
                    coord3D = np.array([l * d]) + l_0
                    ext_points.append(coord3D)
        j += 1
    return np.asarray(ext_points).reshape(-1,3)