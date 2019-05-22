import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
import os
from mpl_toolkits.mplot3d import Axes3D
np.set_printoptions(suppress=True)
def getUname():
    uname = os.getlogin()
    if uname == "trymdh":
        uname = uname + ".WIN-NTNU-NO"
    return uname
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

uname = getUname()

#Camera calibration parameters
ret,K,tvecs,rMats,dist = loadCaliParam(uname)

#Laser plane
os.chdir("C:/Users/" +uname + "/Onedrive/tpk4940Master")
laserPlane = np.load("C:/Users/" + str(uname) + "/OneDrive/tpk4940Master/BestRansacPlane.npy")
lnormal = laserPlane[0:3]
lpoint = laserPlane[3:]/1000


#HandEye and BaseEye
tool_camera = np.load("X.npy")
base_tool = np.load("T.npy")
base_camera = base_tool@tool_camera
base_camera[:3,3] /= 1000

#laser image points
#points = np.load("Points.npy").reshape(-2,1,2)
points = np.loadtxt("laserline_2d.txt").reshape(-2,1,2)

points_norm = cv.undistortPoints(points,K,dist).reshape(-1,2)
points_norm = np.hstack((points_norm,np.ones((2048,1))))
print(points_norm)
print(base_camera)

points_norm_world = (base_camera[:3,:3]@points_norm.T + base_camera[:3,3].reshape(3,1)).T
print(points_norm_world)

#intersections between plane and image ray
l0 = base_camera[:3,3].reshape(3,1)
n = lnormal.reshape(3,1)
p0 = lpoint.reshape(3,1)
x3d = []
for x in points_norm_world:
    l = x.reshape(3,1) - l0
    l /= np.linalg.norm(l)
    l = l.reshape(3,1)
    d1 = np.dot((p0-l0).T,n)
    d2 = np.dot(l.T,n)
    y = (d1/d2)*l + l0
    x3d.append(y)

x3d = np.array(x3d).reshape(-1,3)
#print(x3d)

X = x3d[:,0]
Y = x3d[:,1]
Z = x3d[:,2]

ax = plt.axes(projection='3d')
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
#clean_points = points
#ax.set_aspect("equal")
ax.set_xlim(min(X),max(X))
ax.set_ylim(min(Y),max(Y))
ax.set_zlim(0,max(Z))



#print(max(Z)-min(Z))
ax.scatter3D(X,Y,Z)
plt.show()