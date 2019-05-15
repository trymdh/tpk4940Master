from AT_cx_functions import*
from matplotlib import pyplot as plt 
import os
import numpy as np 

n = 1
for i in range(0,18):
    #VISUALY CHECK THE IMAGE IF THE CORRECT POINTS ARE FOUND
    uname = os.getlogin()
    if uname == "trymdh":
        uname = uname + ".WIN-NTNU-NO"

    laserimage_folder = "C:/Users/" + str(uname) + "/OneDrive/tpk4940Master/Camera calibration May/LaserImages"
    os.chdir(laserimage_folder)

    laserimage = cleanUpLaser(np.load("pixcoord_"+ str(n)+ ".npy"))
    print(laserimage)
    plt.xlim(0,2048)
    plt.ylim(-50,1080)
    plt.gca().invert_yaxis()
    plt.plot(laserimage[:,1],'r')
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