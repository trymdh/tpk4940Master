import os
import numpy as np 

def loadCaliParam(uname):
    """
    This functions loads the camera calibration parameters obtained from Matlab into numpy arrays
    """
    #path to the folder where the parameters are saved
    caliParam_folder = "C:/Users/" + str(uname) + "/OneDrive/tpk4940Master/Camera calibration May/Calibration parameters"
    os.chdir(caliParam_folder)

    #Mean Reprojection Error
    ret = np.loadtxt('MeanReprojectionError.txt')
   
    #The Intrisinc Matrix
    K = np.loadtxt('IntrinsicMatrix.txt') #Old
    
    #Rotation Matrices and translation vectors between the scene and the camera
    tvecs = np.loadtxt('TranslationVectors.txt')
    rMats = np.loadtxt('RotationMatrices.txt') # Note: this file contains all the scene/camera rotationmatrices for each picture. It needs to be reshaped from (#,3) into (#/3,3,3)
    shp = rMats.shape
    C = int(shp[0]/3)
    rMats = rMats.reshape(C,3,3)
    
    #Radial and tangential distortion coeffecients
    dist = []
    rDist = np.loadtxt('RadialDistortion.txt') #k_1, k_2 and k_3 (k_3 = 0)
    tDist = np.loadtxt('TangentialDistortion.txt') #p_1 and p_2
    dist.append(rDist)
    dist.append(tDist)
    dist = np.asarray(dist).reshape(1,4) #dist = [k_1,k_2,p_1,p_2]

    return ret,K,tvecs,rMats,dist