from AT_cx_functions import*
import cx.cx_cam as cam
import cx.cx_base as base 
import time 
from matplotlib import pyplot as plt
import cv2
import os
import glob

#Find and configure the device
hDev = getDevice()

wd = getCurrentWD()
#Create a new image folder if it does not already exist
laserimage_folder = wd + "/Newlaserimage"
if not os.path.exists(laserimage_folder): 
    os.makedirs(laserimage_folder) 
os.chdir(laserimage_folder) 

laserPixelList = os.getcwd().replace("\\","/") + "/*.npy"
laserImageList = os.getcwd().replace("\\","/") + "/*.png"
laserPixelList = sortList(glob.glob(laserPixelList))
laserImageList = sortList(glob.glob(laserImageList))

if cam.cx_getParam(hDev,"CameraMode")[1] != "CenterOfGravity":
    cogConfig(hDev)
    #Snap an image of the scene in COG mode and save the output.
    time.sleep(5)
    laserlinepixels = snap(hDev)
    laserpixel = pixCoordify(laserlinepixels.ravel() / 64 , 2048) #reshape the COG-image from (2048,) to (2048,2)
    if laserPixelList == []:
        save_name = laserimage_folder + "/pixcoord_1.npy"
        np.save(save_name,laserpixel)
    else:
        n = len(laserPixelList) + 1
        save_name = laserimage_folder + "/pixcoord_" + str(n) + ".npy"
        np.save(save_name,laserpixel)

num_img = len(laserImageList)
img_thresh = 20 #number of images needed for a good calibration, n < 20

#Snap an image in image mode and save it
if cam.cx_getParam(hDev,"CameraMode")[1] != "Image":
    imageConfig(hDev)
    time.sleep(5)
    image = snap(hDev)
    if laserImageList == []:
        save_name = laserimage_folder + "/1.png"
        cv2.imwrite(save_name,image)
    elif num_img < img_thresh:
        print("\nNeed "+ str((img_thresh-num_img)) + " more pictures of chessboard")
        #copy the images already in the folder to a list
        images = []
        for fname in laserImageList:
            images.append(cv2.imread(fname))
        images.append(image)
        n = 1
        for img in images:
            save_name = laserimage_folder + "/" + str(n) + ".png"
            cv2.imwrite(save_name, img)
            n = n + 1


