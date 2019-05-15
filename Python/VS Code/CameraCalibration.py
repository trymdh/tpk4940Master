# The purpose of this script is to calibrate the C4-2040 camera using OpenCV
# and the AT SDK.
import numpy
import cv2
import cx.cx_cam as cam
import cx.cx_base as base 
from AT_cx_functions import *
import glob

uname = getUname()

image_folder = "C:/Users/"+ str(uname) + "/OneDrive/tpk4940Master/Camera calibration May/Images"
os.chdir(image_folder)

#image_paths contains the paths to all the images in the calibration folder
image_paths = os.getcwd().replace("\\","/") + "/*.png"
image_paths = sortList(glob.glob(image_paths))

calibration = 1 #1 = Calibrate camera, 0 = do nothing
if calibration == 1: 
    #init the object points of the chessboard
    chess = Chessboard()
    #create arrays to store object points and image points from all the images
    objpoints = [] #3D points in real world space
    imgpoints = [] #2D points in image plane
    
    for fname in image_paths:
        #Load and convert the image to grayscale
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        
        #Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray,(chess.cbcol,chess.cbrow),None)
        
        #If the corners are found, add the object points and image points to the respective lists (after refining them)
        if ret == True:
            objpoints.append(chess.objp)
            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), chess.criteria)
            imgpoints.append(corners2)
            """
            #Draw and display the corners
            img = cv2.drawChessboardCorners(img, (chess.cbcol, chess.cbrow), corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(0)
            """
            
    #cv2.destroyAllWindows()
    
    #ret = mean reprojection error (it should be as close to zero as possible)
    #mtx = the matrix of intrisic parameters
    #dist = the distortion parameters
    #rvecs =  the rotation vectors (one per image)
    #tvecs =  the translation vectors (one per image)
    ret,mtx,dist,rvecs,tvecs = cv2.calibrateCamera(objpoints,imgpoints,gray.shape[::-1],None,None)

    print(len(tvecs))
    print("\n \n {0}".format(rvecs))