import numpy as np
import cvb
import os
import sys
import matplotlib.pyplot as plt
import cv2
import glob
print(sys.version)
print(cvb.version())

#Checkerboard dimensions
cbrow = 6
cbcol = 10

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((cbrow*cbcol,3), np.float32)
objp[:,:2] = np.mgrid[0:cbcol,0:cbrow].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

#import images
images = glob.glob("C:/Users/TrymAsus/Dropbox/Python/Robotikk/cboard/C4ChessImg/*.jpg")

for fname in images:
    img = cv2.imread(fname)

    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (cbcol,cbrow),None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (cbcol,cbrow), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(500)
cv2.destroyAllWindows() 


#ret = mean reprojection error (it should be as close to zero as possible)
#mtx = the matrix of intrisic parameters
#dist = the distortion parameters
#rvecs =  the rotation vectors (one per image)
#tvecs =  the translation vectors (one per image)
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
print(mtx)
print(ret)
print(dist)

#Undistort a test image using the calibrated intrinsic matrix and the estimated
#distortion coeffesients for radial and tangential lensdistortion

img = cv2.imread("C:/Users/TrymAsus/Dropbox/Python/Robotikk/cboard/C4ChessImg/2.jpg")
cv2.imshow('img', img)
cv2.waitKey(0)
img_undist = cv2.undistort(img,mtx,dist)
cv2.imshow('img_undist',img_undist)
cv2.waitKey(0)


# # Acquire images from AT C4 camera


with cvb.DeviceFactory.open(os.environ["CVB"] + "/drivers/GenICam.vin") as device:
    stream = device.stream
    stream.start()
    try:
        for i in range(0,10):
            image, status = stream.wait_for(5000)
            print(image)
            if status == cvb.WaitStatus.Ok:
                print("Acquired image " + str(i) + " into buffer " + 
                   str(image.buffer_index) + ".")
                img_str = 'snap'+str(i)+".bmp"
                image.save(img_str)
            else:
                raise RuntimeError("timeout during wait" 
                    if status == cvb.WaitStatus.Timeout else 
                        "acquisition aborted")
    except:
        pass 
    finally:
        stream.stop()




