from AT_cx_functions import*
from matplotlib import pyplot as plt
import msvcrt
import cv2 as cv

#Find and configure the device
hDev = getDevice()
imageConfig(hDev)
Pressed = True
while Pressed:
    if msvcrt.kbhit():
        key = str(msvcrt.getch()).replace("b'","").replace("'","")
        if key == 'q':
            if cam.cx_getParam(hDev,"CameraMode")[1] != "CenterOfGravity":
                    cogConfig(hDev)
            Pressed = False
            plt.close()
            print("Loop exited")
            break
    #Snap an image of the scene in COG mode.
    img = snap(hDev)
    #scale the image and plot with matplotlib
    img_mean = np.mean(img.data)
    img_min = np.min(img.data)
    img_max = np.max(img.data)
    img_std = np.std(img.data)

    #render image
    plt.clf()
    plt.ion()
    axes_img = plt.imshow(img.data, vmin = img_mean - 3*img_std, vmax = img_mean + 3*img_std, cmap = "gray")
    plt.pause(1/60)