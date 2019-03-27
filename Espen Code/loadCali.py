from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import os
import glob
import cv2 as cv2



def loadCaliParam():
    """
    This functions loads the camera calibration parameters
    obtained from Matlab into numpy arrays
    """
    #path to the folder where the parameters are saved
    #caliParam_folder = "C:/Users/trymdh.WIN-NTNU-NO/OneDrive/Master/Matlab" #work pc
    caliParam_folder = "C:/Users/sioux/OneDrive/Master/Laserplankalibrering/Bilder_og_koordinater/Matlab" # home pc
    os.chdir(caliParam_folder)

    #Mean Reprojection Error
    ret = np.loadtxt('MeanReprojectionError.txt')
    
    #All Reprojection Errors
    rets = np.loadtxt('ReprojectionErrors.txt')

    #The Intrisinc Matrix
    mtx = np.loadtxt('./CalibrationConstants/calibratedCameraMatrix.txt')

    #Rotation Matrices and translation vectors between the scene and the camera
    tvecs = np.loadtxt('TranslationVectors.txt')
    rMats = np.loadtxt('RotationMatrices.txt') # Note: this file contains all the scene/camera rotationmatrices for each picture. It needs to be reshaped from (#,3) into (#/3,3,3)
    shp = rMats.shape
    C = int(shp[0]/3)
    rMats = rMats.reshape(C,3,3)

    #Radial and tangential distortion coeffecients, dist = [k_1,k_2,p_1,p_2[,k_3[,k_4,k_5,k_6]]]
    dist = []
    rDist = np.loadtxt('./CalibrationConstants/calibratedRaddist.txt') #k_1 and k_2, => k_3 = 0, this leads to dist = [k_1,k_2,p_1,p_2]
    tDist = np.loadtxt('./CalibrationConstants/calibratedTangdist.txt') #p_1 and p_2
    dist.append(rDist)
    dist.append(tDist)
    dist = np.asarray(dist).reshape(1,4)
    
    #Load rotational and translational errors

    rotVecError = np.loadtxt('rotVecError.txt')
    transVecError = np.loadtxt('transVecError.txt')

    return ret,rets,mtx,tvecs,rMats,dist,rotVecError,transVecError