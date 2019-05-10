import re
import os
import glob
import cv2
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
"""
This script estimates the laser plane in camera coordinates based on the RANSAC algorithm.
Input to the RANSAC function is a 3D pointcloud of the laserpoints as seen from the camera
frame. 

The laser pointcloud is calculated based on calibration parameters from the 
intrinsic camera calibration done in MATLAB. The parameters are exported to Python via loadCaliParam(). The 
and fed to LaserPointsCloud() which triangulates the laser points and returns a pointcloud
of all the triangulated laser points.
"""
def sortList(unsortedList):
    #sort a list in alphanumeric order
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)',key)]
    return sorted(unsortedList,key = alphanum_key)

def loadCaliParam(uname):
    """
    This functions loads the camera calibration parameters obtained from Matlab into numpy arrays
    """
    #path to the folder where the parameters are saved
    if str(uname) == "trymdh":
        uname = uname + ".WIN-NTNU-NO"
    caliParam_folder = "C:/Users/" + str(uname) + "/OneDrive/tpk4940Master/Camera calibration May/Calibration parameters"
    os.chdir(caliParam_folder)

    #Mean Reprojection Error
    ret = np.loadtxt('MeanReprojectionError.txt')
   
    #The Intrisinc Matrix
    mtx = np.loadtxt('IntrinsicMatrix.txt') #Old
    
    #Rotation Matrices and translation vectors between the scene and the camera
    tvecs = np.loadtxt('TranslationVectors.txt')
    rMats = np.loadtxt('RotationMatrices.txt') # Note: this file contains all the scene/camera rotationmatrices for each picture. It needs to be reshaped from (#,3) into (#/3,3,3)
    shp = rMats.shape
    C = int(shp[0]/3)
    rMats = rMats.reshape(C,3,3)
    
    
    #Radial and tangential distortion coeffecients, dist = [k_1,k_2,p_1,p_2[,k_3[,k_4,k_5,k_6]]]
    dist = []
    rDist = np.loadtxt('RadialDistortion.txt') #k_1 and k_2, => k_3 = 0, this leads to dist = [k_1,k_2,p_1,p_2]
    tDist = np.loadtxt('TangentialDistortion.txt') #p_1 and p_2
    dist.append(rDist)
    dist.append(tDist)
    dist = np.asarray(dist).reshape(1,4)

    return ret,mtx,tvecs,rMats,dist

def LaserPointsCloud(uname,K,dist):
    if str(uname) == "trymdh":
        uname = uname + ".WIN-NTNU-NO"
    os.chdir("C:/Users/" + str(uname) + "/OneDrive/tpk4940Master/Camera calibration May/LaserImages")
    laser_fnames_list = sortList(glob.glob(os.getcwd().replace("\\","/") + "/*.npy"))
    laserImages = []
    for fname in laser_fnames_list:
        laserImages.append(np.load(fname))
    
    laserImgs = np.asarray(laserImages)
    ext_points = []
    l_0 = np.array([0,0,0])
    j = 0
    for pix in laserImgs:
        pix = pix.reshape(-2,1,2)
        for pxls in pix:
            if pxls[0][1] != 0:
                pxls = pxls[0].reshape(-2,1,2)
                R = R_mats[j]
                t = t_vecs[j]
                T = np.eye(4)
                T[0:3,0:3] = R
                T[0:3,3] = t
                n = R[2,:] #this might be wrong, if wrong then n = R[:,2]
                p_0 = t
                undist_pix = cv2.undistortPoints(pxls,K,dist).reshape(-1,2)# undistorted normalized image coordinate
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

def ransacPlane(pointCloud):
    bestFit = None
    bestErr = np.inf
    centroid = None
    best_cnt_in = 0
    goal_inlier = 0.7*len(pointCloud)
    N = np.inf
    ite = 0
    while ite < N:
        #sample 3 random points and estimate plane
        maybeIndex = np.random.choice(pointCloud.shape[0],3,replace = False)
        maybeInliers = pointCloud[maybeIndex,:]
        pI,c = svd_AxB(homogenify(maybeInliers))
        #calculate error and inlier threshold
        error_vec,median_error,error_std = getError(pointCloud,pI,c)
        #count inliers
        Inliers = countInliers(error_vec, median_error,error_std,pointCloud)
        cnt_in = len(Inliers)
        if cnt_in >= goal_inlier:
            betterData = Inliers
            #The new dataset contains few outliers
            betterFit,c = svd_AxB(homogenify(betterData))
            error = error_checker(betterFit,pointCloud)
            if (cnt_in >= best_cnt_in) and (error < bestErr):
                #update N
                w = cnt_in/len(pointCloud)
                N = np.log(1-0.99)/np.log(1-w**3)
                #Update best fit plane
                bestFit = betterFit
                best_cnt_in = cnt_in
                bestErr = error
                centroid = c
        ite += 1
    return bestFit,centroid,bestErr

def countInliers(error_vec, median_error,error_std,pointCloud):
    i = 0
    cnt_in = 0
    Inliers = []
    for error in error_vec:
        if np.abs(error) < np.abs(median_error) + 0.5*np.abs(error_std):
        #if np.abs(error) < 0.2:
            cnt_in += 1
            Inliers.append(pointCloud[i])
        i += 1
    Inliers = np.array(Inliers)
    return Inliers

def homogenify(G):
    H = []
    for point in G:
        H.append(np.append(point,1))
    return np.asarray(H)

def svd_AxB(pointCloud):
    u,s,vh = np.linalg.svd(pointCloud)
    v = vh.conj().T
    x = v[:,-1]
    x = x.T.reshape(-1)
    c = getCentroid3D(pointCloud)
    n = x[0:3]
    d = x[3]/np.linalg.norm(n)
    #[A,B,C,D]
    pI = x*d
    return pI,c

def getCentroid3D(pointCloud):
    #this function returns the centroid
    xs = pointCloud[:,0]; ys = pointCloud[:,1]; zs = pointCloud[:,2]
    x_av = np.average(xs); y_av = np.average(ys); z_av = np.average(zs)

    return np.array([x_av,y_av,z_av])

def error_checker(plane,point_cloud):
    [A,B,C,D] = plane
    distances = np.array([])
    for i in range(len(point_cloud[:,0])):
        [x,y,z] = [point_cloud[i,0],point_cloud[i,1],point_cloud[i,2]] 
        d = abs(A*x + B*y + C*z + D)/np.sqrt(A**2 + B**2 + C**2)
        distances = np.append(distances,d)
    return sum(distances)/len(distances)

def getError(pointCloud,pI,c):
    [A,B,C,D] = pI
    error_list = []
    for point in pointCloud:
        d = (A*(point[0]-c[0]) + B*(point[1]-c[1]) + C*(point[2]-c[2]))/np.sqrt(A**2 + B**2 + C**2)
        error_list.append(d)
    error_vec = np.array(error_list)
    median_error = np.median(error_vec)
    error_std = np.std(error_vec)
    #error_vec contains distances between each point in pointCloud and the plane pI
    return error_vec,median_error,error_std

def getPlaneData(pI,ax):
    #This function takes the plane parameters and the matplotlib plot object as input 
    #and returns the data values needed to plot a wireframe using plt.subplot.plot_wireframe()
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),np.arange(ylim[0], ylim[1]))
    Z = np.zeros(X.shape)
    for r in range(X.shape[0]):
        for c in range(X.shape[1]):
            #z = a*X + b*Y + d 
            Z[r,c] = -(pI[0] * X[r,c] + pI[1] * Y[r,c] + pI[3])*1./pI[2]

    return X,Y,Z


#get the system username, usefull when working on different computers...
uname = os.getlogin()

#Load camera parameters
ret,K,t_vecs,R_mats,dist = loadCaliParam(uname)
K_inv = np.linalg.inv(K)

pointCloud = LaserPointsCloud(uname,K,dist)

bestFit,c,bestErr = ransacPlane(pointCloud)
print("Ransac plane: {0} \nError: {1}".format(bestFit,bestErr))

#plot laser points
ext_points = ext_points[::50]
x = pointCloud[:,0]
y = pointCloud[:,1]
z = pointCloud[:,2]

#plot ransac plane
plt.figure(1)
ax = plt.subplot(111, projection ='3d')
ax.scatter(x, y, z, color ='b')
X_r,Y_r,Z_r = getPlaneData(bestFit,ax)
ax.plot_wireframe(X_r,Y_r,Z_r, color='g', alpha = 0.5)
plt.show()
