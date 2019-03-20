import os 
import numpy as np
import cv2
from AT_cx_functions import*

ret,mtx,tvecs,rMats,dist = loadCaliParam()
print(mtx)
#os.chdir('C:/Users/trymdh.WIN-NTNU-NO/OneDrive/Master/VS Code/laserimage') #work pc
os.chdir('C:/Users/Trym/OneDrive/Master/VS Code/laserimage') #home pc
wd = getCurrentWD()

img = cv2.imread('11.png')
cv2.imshow('img',img)

img_undist = cv2.undistort(img,mtx,dist)
cv2.imshow('undist',img_undist)
cv2.imwrite('undist_11.png',img_undist)
cv2.waitKey(0)
cv2.destroyAllWindows()
