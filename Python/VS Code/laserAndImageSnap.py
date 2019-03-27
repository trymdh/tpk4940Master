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
cogConfig(hDev)

wd = getCurrentWD()
laserimage_folder = wd + "/laserimage"
if not os.path.exists(laserimage_folder): 
    os.makedirs(laserimage_folder) 
os.chdir(laserimage_folder) 

#Snap an image of the scene in COG mode and save the output.
laserlinepixels = snap(hDev)
laserpixel = pixCoordify(laserlinepixels.ravel() / 64 , 2048) #reshape the COG-image from (2048,) to (2048,2)

laserPixelList = os.getcwd().replace("\\","/") + "/*.npy"
laserImageList = os.getcwd().replace("\\","/") + "/*.png"
laserPixelList = glob.glob(laserPixelList)
laserImageList = glob.glob(laserImageList)

num_img = len(laserImageList)
img_thresh = 20 #number of images needed for a good calibration, n < 20

if laserPixelList == []:
    save_name = wd + "/laserimage/pixcoord_1.npy"
    np.save(save_name,laserpixel)
else:
    n = len(laserPixelList) + 1
    save_name = wd + "/laserimage/pixcoord_" + str(n) + ".npy"
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


