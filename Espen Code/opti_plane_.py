from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import os
import glob
from scipy.optimize import minimize
from comparisonfunctions import error_checker
from comparisonfunctions import fdiff_checker
from weightAssigner import weights



def loadCaliParam():
    """
    This functions loads the camera calibration parameters
    obtained from Matlab into numpy arrays
    """
    #path to the folder where the parameters are saved
    #caliParam_folder = "C:/Users/trymdh.WIN-NTNU-NO/OneDrive/Master/Matlab" #work pc
    caliParam_folder = "C:/Users/Espen/OneDrive/Master/Laserplankalibrering/Bilder_og_koordinater/Matlab" # home pc
    os.chdir(caliParam_folder)

    #Mean Reprojection Error
    ret = np.loadtxt('MeanReprojectionError.txt')

    #All Reprojection Errors
    rets = np.loadtxt('ReprojectionErrors.txt')

    #The Intrisinc Matrix
    mtx = np.loadtxt('IntrinsicMatrix.txt')

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

    return ret,rets,mtx,tvecs,rMats,dist



ret,rets,K,tvecs,rMats,dist = loadCaliParam()

K_inv = np.linalg.inv(K)
os.chdir('C:/Users/Espen/OneDrive/Master/Laserplankalibrering/Bilder_og_koordinater')
laser_npy = os.getcwd().replace("\\","/") + "/*.npy"
laser_npy = glob.glob(laser_npy)
number_of_laserfiles = len(laser_npy)
#print(number_of_laserfiles)
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
"""
ext_points = ext_points[2048:]
ext_points = ext_points[::100]
"""
x = ext_points[:,0]
y = ext_points[:,1]
z = ext_points[:,2]

#QP solver takes in the extracted points in addition to the Weights of the different values
def QP_solver(ext_points,W):
    #Define Centroid of pointcloud
    x = ext_points[:,0]
    y = ext_points[:,1]
    z = ext_points[:,2]
    [cx, cy, cz] = np.array([sum(x)/len(x),sum(y)/len(y),sum(z)/len(z)])
    #Distance from each point to centroid
    [mx,my,mz] = np.array([(x-cx).reshape(1,-1),(y-cy).reshape(1,-1),(z-cz).reshape(1,-1)])

    D = np.array([mx,my,mz]).reshape(3,-1) 
    D_T = np.transpose(D)
    Q = D@D_T

    def cfunc2(n):
        return np.transpose(n)@Q@n
    def sum_func(n):
        summ = 0
        plain_sum = 0
        for i in range(len(D[1,:])):
            summ += (D[0,i]*n[0] + D[1,i]*n[1] + D[2,i]*n[2])**2
            plain_sum += abs(D[0,i]*n[0] + D[1,i]*n[1] + D[2,i]*n[2])
            if i == 0:
                print("sum : ", summ)
        #print(summ,n)
        #print("Avg error : ",plain_sum/len(D[1,:]))
        return summ
    def cfunc(n):
        return (sum_func(n))
    def constraint(n):
        return 1 - n[0]**2 - n[1]**2 - n[2]**2 
    const = ({'type': 'eq', 'fun':constraint})
    n0 = np.array([1,1,1]) #Initial guess of n-vector
    
    if W == True:
        print("Solving with weights")
        sol = minimize(cfunc,n0,method='SLSQP',constraints=const,options={'disp':True})
    else:
        print("Solving without weights")
        sol = minimize(cfunc2,n0,method='SLSQP',constraints=const,options={'disp':True})
    return sol,cx,cy,cz

[sol,cx,cy,cz] = QP_solver(ext_points,0)

nOpt = sol.x
value = sol.fun
print("n: ",nOpt, "c: ", cx,"," , cy,"," ,cz)

"""
fra den andre : 3.362587 x + 10.409694 y + 399.928753 - z= 0 evt [3.362587,10.409694,399.928753,-1]"
"""
plane_LS = [3.362587,10.409694,399.928753,-1]

raw = [23030113428831100,70634739100933070, 5609153211116791, 1370979959123711000] #Ax+By+Cz+D=0
#Into Ax +By + D = z

plane_QP = [nOpt[0],nOpt[1],nOpt[2],-nOpt[0]*cx-nOpt[1]*cy-nOpt[2]*cz]

error_LS = error_checker(plane_LS,ext_points)
error_QP = error_checker(plane_QP,ext_points)

print("Tot plane-normal error LS :",error_LS," ", "Tot plane-normal error QP :", error_QP)

zerror_LS = fdiff_checker(plane_LS,ext_points)
zerror_QP = fdiff_checker(plane_QP,ext_points)

print("Tot fval error LS :",zerror_LS," ", "Tot fval error QP :", zerror_QP)

QP_weights = weights(rets)
