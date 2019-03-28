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
import cv2 as cv2
from loadCali import loadCaliParam
from planeify import planeify

#Import values from calibration process
ret,rets,K,tvecs,rMats,dist,rotVecError,transVecError = loadCaliParam()

K_inv = np.linalg.inv(K)
os.chdir('C:/Users/sioux/OneDrive/Master/Laserplankalibrering/Bilder_og_koordinater/LaserAndNpys')
laser_npy = os.getcwd().replace("\\","/") + "/*.npy"
laser_npy = glob.glob(laser_npy)
number_of_laserfiles = len(laser_npy)
#print(number_of_laserfiles)
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

#W = weights(rets,None,'Repr')
W = weights(rotVecError,transVecError,'Vecs')
print('Weights are :',W)



[sol,cx_,cy_,cz_] = QP_solver(pixel_dict,W)


#os.chdir('C:/Users/sioux/OneDrive/Master/Laserplankalibrering/Bilder_og_koordinater')
#np.save('laserplane.npy',[sol.x[0],sol.x[1],sol.x[2],cx_,cy_,cz_])

print("n: ",sol.x, "c: ", cx_,"," , cy_,"," ,cz_)


plane_QP = [sol.x[0],sol.x[1],sol.x[2],-sol.x[0]*(cx_)-sol.x[1]*(cy_)-sol.x[2]*(cz_)]
print(plane_QP)
plane_raw = [sol.x[0],sol.x[1],sol.x[2],cx_,cy_,cz_]

[p,ps] = planeify(plane_raw)

print(ps)

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