from AT_cx_functions import*
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cv2
import os
import numpy as np 

ret,K,tvecs,rMats,dist = loadCaliParam()
n = 1
for i in range(0,18):
    R = rMats[n-1]
    t = tvecs[n-1]
    N = R[2,:]
    l_0 = np.array([0,0,0])
    #VISUALY CHECK THE IMAGE IF THE CORRECT POINTS ARE FOUND
    uname = os.getlogin()
    if uname == "trymdh":
        uname = uname + ".WIN-NTNU-NO"
    laserimage_folder = "C:/Users/" + str(uname) + "/OneDrive/tpk4940Master/Camera calibration May/LaserImages"
    os.chdir(laserimage_folder)
    laserimage = cleanUpLaser(np.load("pixcoord_"+ str(n)+ ".npy"))
    point_3D = []
    for pix in laserimage:
        norm_img_coord = cv2.undistortPoints(pix.reshape(-2,1,2),K,dist)
        norm_img_coord = np.append(norm_img_coord.reshape(-1,2),1)
        l = norm_img_coord/np.linalg.norm(norm_img_coord)
        d = np.dot((t - l_0),N)/np.dot(l,N)
        point_3D.append(np.array([l * d]) + l_0)
    point_3D = np.asarray(point_3D).reshape(-1,3)
    x = point_3D[:,0]
    y = point_3D[:,1]
    z = point_3D[:,2]
    plt.figure()
    ax = plt.subplot(111,projection="3d")
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.scatter(x, y, z, color ='b')
    plt.show()

    image_folder = "C:/Users/" + str(uname) + "/OneDrive/tpk4940Master/Camera calibration May/Images"
    os.chdir(image_folder)
    img = cv2.imread(str(n) + ".png")
    #plot the points on the image to check if the laserline is found
    for i in laserimage:
        cx,cy = i
        if np.isnan(cy):
            #cv2 can't plot points with NAN coordinates
            cy = 0
        elif cy != 0: 
            #only want to plot the found points, i.e the points that are not 0
            cv2.circle(img,(int(cx),int(cy)),1,(0,0,255))
    cv2.imshow("img", img)
    cv2.waitKey()
    n += 1