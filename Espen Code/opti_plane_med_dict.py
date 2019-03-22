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
pixel_dict = {} #Sorts each extracted point in a dictionary based on image such that weights can be correctly assigned
for i in range(1,len(laser_npy) + 1):
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
            pixel_dict[str(i)] = np.append(pixel_dict[str(i)],fullcoord)
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
def QP_solver(dictio,W):
    #Define Centroid of pointcloud
    x_values = np.array([])
    y_values = np.array([])
    z_values = np.array([])
    for key,val in dictio.items(): #Apply weigths here aswell?
        x_values = np.append(x_values,val[0::3])
        y_values = np.append(y_values,val[1::3])
        z_values = np.append(z_values,val[2::3])   

    [cx_, cy_, cz_] = np.array([sum(x_values)/len(x_values),sum(y_values)/len(y_values),sum(z_values)/len(z_values)])



    #Distance from each point to centroid
    centroid_vectors = {}
    for key,val in dictio.items():
        centroid_vectors[key] = np.array([(val[0::3]-cx_),(val[1::3]-cy_),(val[2::3]-cz_)])



    x = ext_points[:,0]
    y = ext_points[:,1]
    z = ext_points[:,2]
    [cx, cy, cz] = np.array([sum(x)/len(x),sum(y)/len(y),sum(z)/len(z)])
    [mx,my,mz] = np.array([(x-cx).reshape(1,-1),(y-cy).reshape(1,-1),(z-cz).reshape(1,-1)])
    D = np.array([mx,my,mz]).reshape(3,-1) 
    D_T = np.transpose(D)
    Q = D@D_T
    """
    print((len(centroid_vectors['20'][2,:]) + len(centroid_vectors['19'][2,:]) + len(centroid_vectors['18'][2,:]) + len(centroid_vectors['17'][2,:]) + len(centroid_vectors['16'][2,:]) + len(centroid_vectors['15'][2,:]) + len(centroid_vectors['14'][2,:]) + len(centroid_vectors['13'][2,:]) + len(centroid_vectors['12'][2,:]) + len(centroid_vectors['11'][2,:]) + len(centroid_vectors['10'][2,:]) + len(centroid_vectors['9'][2,:]) + len(centroid_vectors['8'][2,:]) + len(centroid_vectors['7'][2,:]) +  + len(centroid_vectors['6'][2,:]) +  + len(centroid_vectors['5'][2,:]) + len(centroid_vectors['4'][2,:]) + len(centroid_vectors['3'][2,:]) + len(centroid_vectors['2'][2,:]) + len(centroid_vectors['1'][2,:])))
    print(mz)
    print("sum: ",sum(mz))   
    print(centroid_vectors['1'])
    print(mx)  
    
    print(centroid_vectors['1'][0,0])
    print(centroid_vectors['1'][1,0])
    print(centroid_vectors['1'][2,0])
    print(mx[0])
    print(my[0])
    print(mz[0])
    """
    def sum_func(n):
        summ = 0
        plain_sum = 0
        for i in range(len(D[1,:])): 
            summ += (D[0,i]*n[0] + D[1,i]*n[1] + D[2,i]*n[2])**2
            plain_sum += abs(D[0,i]*n[0] + D[1,i]*n[1] + D[2,i]*n[2])
        return summ
    def sum_func_W(n):
        summ = 0
        plain_sum = 0
        for key,val in centroid_vectors.items(): ### HERRRR JA, HENT VERDIA FRA DICT ISTEDE OG ASSIGN VEKTE SÅNN
            for k in range(len(val[1,:])):
                summ += W[key]*(val[0,k]*n[0] + val[1,k]*n[1] + val[2,k]*n[2])**2
                
                
            #print(summ,n)
            #print("Avg error : ",plain_sum/len(D[1,:]))
        return summ
    def cfunc(n):
        return (sum_func_W(n))
    def cfunc2(n):
        return np.transpose(n)@Q@n
    def constraint(n):
        return 1 - n[0]**2 - n[1]**2 - n[2]**2 #Vectorlength = 1
    const = ({'type': 'eq', 'fun':constraint})

    n0 = np.array([1,1,1]) #Initial guess of n-vector
    if W:
        sol = minimize(cfunc,n0,method='SLSQP',constraints=const,options={'disp':True})
        print("Solving with weights")
    else:
        sol = minimize(cfunc2,n0,method='SLSQP',constraints=const,options={'disp':True}) 
        print("Solving without weights") 

    return sol,cx_,cy_,cz_

#[sol,cx,cy,cz] = QP_solver(ext_points,0)
"""
a = [100,1,1,1,1,1,6,1,1,1,1,1,1,1,1,1,1,1,1,1]
factor = 0.001
W = [x*factor for x in a]
"""
W = weights(rets)
print(W)
print(W['1'])


#n:  [ 0.35647423  0.9333939  -0.04125469] c:  43.567485091817716 , 39.68287203035959 , 1111.3939797585415

#n:  [ 0.35647419  0.93339391 -0.04125484] c:  43.567485091817716 , 39.68287203035959 , 1111.3939797585415

[sol,cx_,cy_,cz_] = QP_solver(pixel_dict,W)
#print(W)

#value = sol.fun
print("n: ",sol.x, "c: ", cx_,"," , cy_,"," ,cz_)


#fra den andre : 3.362587 x + 10.409694 y + 399.928753 - z= 0 evt [3.362587,10.409694,399.928753,-1]
"""
plane_LS = [3.362587,10.409694,399.928753,-1]
[ 0.30903221  0.94811301 -0.07470478] c:  41.24233446925988 , 53.650329399124125 , 1099.2964489066349
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
"""