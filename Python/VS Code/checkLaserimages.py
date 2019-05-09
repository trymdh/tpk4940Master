    
from AT_cx_functions import*
import os
from matplotlib import pyplot as plt
import cv2 as cv

def cleanUpLaser(unclean_laserpixel):
    clean_laser = []
    for point in laserpixel:
        if point[1] == 0:
            point[1] = None
        clean_laser.append(point)
    return np.asarray(clean_laser)

laserimage_folder = "C:/Users/trymdh.WIN-NTNU-NO/OneDrive/tpk4940Master/NewLaserImages"
os.chdir(laserimage_folder)

laserpixel = np.load("pixcoord_3.npy")
#clean_laserpixel = cleanUpLaser(laserpixel)

plt.xlim(0,2048)
plt.ylim(-50,1080)
plt.gca().invert_yaxis()
plt.plot(laserpixel[:,1],'r')
plt.show()

#plot the points on the image to check if the laserline is found
img = cv.imread("3.png")
for i in laserpixel:
    cx,cy = i
    if cy != 0:
        #only want to plot the found points, i.e the points that are not 0
        cv2.circle(img,(int(cx),int(cy)),1,(0,0,255))
cv.imshow("img", img)
cv.waitKey()
cv.imwrite("laserlineOnImage.png",img)