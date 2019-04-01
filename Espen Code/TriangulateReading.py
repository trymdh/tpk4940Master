import numpy as np
from loadCali import loadCaliParam
import cv2 as cv2
import time

def triang(pix):
    t = time.time()
    ret,rets,K,tvecs,rMats,dist,rotVecError,transVecError = loadCaliParam()
    laser = np.load('C:/Users/sioux/OneDrive/Master/Laserplankalibrering/Bilder_og_koordinater/laserplane.npy')
    
    #1 Undistort the pixels
    pix = pix.reshape(-2, 1, 2)
    undistorted_pixels = cv2.undistortPoints(pix, K, dist).reshape(-1,2)

    #2 Triangulate
    ext_points = np.array([])

    #Define lasernormal and laserpoint
    ln = laser[0:3]
    lp = laser[3:6]

    l_0 = np.array([0,0,0])

    for coord in undistorted_pixels:
        coord = np.append(coord,1)
        l = coord/np.linalg.norm(coord)
        num = np.dot((lp - l_0), ln)
        deno = np.dot(l,ln)
        d = num/deno
        fullcoord = np.array([l * d]) + l_0
        ext_points = np.append(ext_points,fullcoord) 
    elapsed = time.time() - t

    return np.reshape(ext_points,(-1,3))