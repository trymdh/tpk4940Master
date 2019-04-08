from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import os
import glob
from AT_cx_functions import*

def planeify(vector_plane): #Assumes form [nx,ny,nz,cx,cy,cz]
    #[A,B,C,D]
    plane = [vector_plane[0],vector_plane[1],vector_plane[2],-vector_plane[0]*(vector_plane[3])-vector_plane[1]*(vector_plane[4])-vector_plane[2]*(vector_plane[5])] 
    #Or on form [Ax + By + D] = z
    plane_s = [-plane[0]/plane[2],-plane[1]/plane[2],-plane[3]/plane[2]]
    return plane,plane_s

def error_checker(plane,point_cloud):
    [A,B,C,D] = plane
    distances = np.array([])
    for i in range(len(point_cloud[:,0])):
        [x,y,z] = [point_cloud[i,0],point_cloud[i,1],point_cloud[i,2]] 
        d = abs(A*x + B*y + C*z + D)/np.sqrt(A**2 + B**2 + C**2)
        distances = np.append(distances,d)
    return sum(distances)/len(distances)

#load all the calibration parameters
ret,K,tvecs,rMats,dist = loadCaliParam()

#Old images
#os.chdir('C:/Users/Trym/OneDrive/Master/VS Code/laserimage') #home pc
#os.chdir('C:/Users/trymdh.WIN-NTNU-NO/OneDrive/Master/VS Code/laserimage') #work pc
#os.chdir('C:/Users/TrymAsus/OneDrive/Master/VS Code/laserimage') #LAPTOP

#New
#os.chdir('C:/Users/Trym/OneDrive/tpk4940Master/Espen Code/LaserAndNpys') #home pc
os.chdir('C:/Users/trymdh.WIN-NTNU-NO/OneDrive/tpk4940Master/Espen Code/LaserAndNpys') #work pc
#os.chdir('C:/Users/TrymAsus/OneDrive/tpk4940Master/Espen Code/LaserAndNpys') #LAPTOP

laser_npy = os.getcwd().replace("\\","/") + "/*.npy"
laser_npy = sortList(glob.glob(laser_npy))
ext_points = extractPoints(laser_npy,rMats,tvecs,K,dist)

ext_points = np.reshape(ext_points,(-1,3))
#ext_points = ext_points[::100]

#RANSAC--------------------------------------------------------------
ransac_fit,c,res = ransacPlane(ext_points)
print(ransac_fit)
RANSACPLANE = np.append(ransac_fit[0:3],c)
np.save('C:/Users/trymdh.WIN-NTNU-NO/OneDrive/tpk4940Master/Python/VS Code/RansacPlane.npy',RANSACPLANE)
EspenLaser = np.load('C:/Users/trymdh.WIN-NTNU-NO/OneDrive/tpk4940Master/Espen Code/laserplane.npy')
print(EspenLaser)

EspenLaser = np.load('C:/Users/trymdh.WIN-NTNU-NO/OneDrive/tpk4940Master/Espen Code/laserplane.npy')
print(EspenLaser)
"""
EspenPlan,EspenPlan_s = planeify(EspenLaser)
espenError = error_checker(EspenPlan,ext_points)
ransacError = error_checker(ransac_fit[0:4],ext_points)


print("EspenError: {0}".format(espenError))
print("RansacError: {0}".format(ransacError))
"""