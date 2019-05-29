import os
import time 
import cv2
import glob
import numpy as np
from matplotlib import pyplot as plt
from PointArrayFunctions import cleanUpLaser
from UtilityFunctions import getUname, getCurrentWD, sortList
from CameraFunctions import pixCoordify, getDevice, cogConfig, imageConfig, snap, cam

#Find and configure the device
hDev = getDevice()
cogConfig(hDev)

wd = getCurrentWD()
laserimage_folder = wd + "/NewLaserImages"
if not os.path.exists(laserimage_folder): 
    os.makedirs(laserimage_folder) 
os.chdir(laserimage_folder) 

#Snap an image of the scene in COG mode and save the output.
laserlinepixels = snap(hDev)
laserpixel = pixCoordify(laserlinepixels.ravel() / 64 , 2048) #reshape the COG-image from (2048,) to (2048,2)

laserPixelList = os.getcwd().replace("\\","/") + "/*.npy"
laserImageList = os.getcwd().replace("\\","/") + "/*.png"
laserPixelList = sortList(glob.glob(laserPixelList))
laserImageList = sortList(glob.glob(laserImageList))


num_img = len(laserImageList)
img_thresh = 20 #number of images needed for a good calibration, n < 20

if laserPixelList == []:
    save_name = laserimage_folder + "/pixcoord_1.npy"
    np.save(save_name,laserpixel)
else:
    n = len(laserPixelList) + 1
    save_name = laserimage_folder + "/pixcoord_" + str(n) + ".npy"
    np.save(save_name,laserpixel)

#plt.gca().invert_yaxis()
#plt.plot(laserpixel)
#plt.show()

#Snap an image in image mode and save it
if cam.cx_getParam(hDev,"CameraMode")[1] != "Image":
    imageConfig(hDev)
    time.sleep(5)

    if laserImageList == []:
        save_name = laserimage_folder + "/1.png"
        image = snap(hDev)
        cv2.imwrite(save_name,image)
    
    elif num_img < img_thresh:

        print("\nNeed "+ str((img_thresh-num_img)) + " more pictures of chessboard")
        #copy the images already in the folder to a list
        images = []
        for fname in laserImageList:
            images.append(cv2.imread(fname))
        image = snap(hDev)
        images.append(image)
        n = 1
        for img in images:
            save_name = laserimage_folder + "/" + str(n) + ".png"
            cv2.imwrite(save_name, img)
            n = n + 1
            
    if cam.cx_getParam(hDev,"CameraMode")[1] != "CenterOfGravity":
        cogConfig(hDev)

#VISUALY CHECK THE IMAGE IF THE CORRECT POINTS ARE FOUND
uname = getUname
laserimage_folder = "C:/Users/" + str(uname) + "/OneDrive/tpk4940Master/NewLaserImages"
os.chdir(laserimage_folder)

Ls = sortList(glob.glob(os.getcwd().replace("\\","/") + "/*.npy")) 
laserimage = cleanUpLaser(np.load(Ls[int(len(Ls)) - 1]))
fname_img = str(int(len(Ls))) + ".png"
img = cv2.imread(fname_img)
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