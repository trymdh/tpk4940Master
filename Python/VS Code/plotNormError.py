from AT_cx_functions import*
import numpy as np 
import glob
import os 

def loadCaliParam():
    """
    This functions loads the camera calibration parameters
    obtained from Matlab into numpy arrays
    """
    #path to the folder where the parameters are saved
    caliParam_folder = "C:/Users/trymdh.WIN-NTNU-NO/OneDrive/tpk4940Master/Espen Code/Matlab" #work pc
    #caliParam_folder = "C:/Users/sioux/OneDrive/Master/Laserplankalibrering/Bilder_og_koordinater/Matlab" # home pc
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

#Import values from calibration process
ret,rets,K,tvecs,rMats,dist,rotVecError,transVecError = loadCaliParam()

K_inv = np.linalg.inv(K)
os.chdir('C:/Users/trymdh.WIN-NTNU-NO/OneDrive/tpk4940Master/Espen Code/LaserAndNpys')
laser_npy = os.getcwd().replace("\\","/") + "/*.npy"
laser_npy = glob.glob(laser_npy)
number_of_laserfiles = len(laser_npy)

ext_points = np.array([])

j = 0
pixel_dict = {} #Sorts each extracted point in a dictionary based on image such that weights can be correctly assigned
for i in range(1,len(laser_npy)):
    pixel_dict.update({str(i):np.array([])})
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
    
    
    k = 0
    pure_coords = np.array([])
    for reading in pix_coord:
        if reading[1] != 0 and 200<=k<=1700 : #Remove pixels beond 1700 due to noise
                pix = [reading[0],reading[1]]
                pure_coords = np.append(pure_coords,pix)
        k += 1 
    
    pix_coord = pure_coords.reshape(-2, 1, 2)
    #Undistort the pixels from the file
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
            pixel_dict[str(i)] = np.append(pixel_dict[str(i)],fullcoord)
        else:
            print("Coord is zero-equivalent")
    j = j + 1

ext_points = np.reshape(ext_points,(-1,3))

os.chdir('C:/Users/trymdh.WIN-NTNU-NO/OneDrive/tpk4940Master/Python/VS Code')
plane = np.load('BestRansacPlane.npy') 
c = plane[3:6]
plane, plane_s = planeify(plane)
error_vec,median_err,std_err = getError(ext_points,plane,c)


plotError2(error_vec,median_err,std_err)
