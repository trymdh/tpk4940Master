import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import cx.cx_cam as cam
import cx.cx_base as base
import cv2
import os
import re
import time
import random
import msvcrt

#CAMERA FUNCTIONS
#-----------------------------------------------------------------------------------------------------------------------------------
def getDevice():
    """
    Finds connected device and assign it to a device handle, i.e hDev = device handle
    The "result" variable is set to base.CX_STATUS_OK if found, see 
    "checkStatusCode()" for more info on the different statuses.
    """
    result = cam.cx_dd_findDevices("", 2000, cam.CX_DD_USE_GEV | cam.CX_DD_USE_GEV_BROADCAST)
    
    #Number of connected devices
    result, num_dev = cam.cx_dd_getNumFoundDevices()
    
    if num_dev == 0:
        print("Error, no cameras found")
        return 0
    elif result == base.CX_STATUS_OK and num_dev != 0:
        #if device is found, get device uri
        result, uri = cam.cx_dd_getParam(0,"URI")
        print(str(num_dev) + " camera found\nURI: "+ uri)    
        #connect device URI of the first discovered device
        result, hDev = cam.cx_openDevice(uri)
        return hDev

def snap(hDev):
    """
    The following steps are necessary to snap a single image.
    1. (Find and) connect a camera device = getDevice()
    2. Allocate and queue internal buffers.
    3. Start acquisition.
    4. Grab an image buffer.
    5. Save the image
    6. Queue back the image buffer.
    7. Stop acquisition.
    8. Cleanup.
    """
    # 2. Allocate and queue internal buffers.
    result =  cam.cx_allocAndQueueBuffers(hDev, 16)
    if result!=base.CX_STATUS_OK:
        print("cx_allocAndQueueBuffers(%d) returned error %d" % (hDev, result))

    # 3. Start acquisition.
    result =  cam.cx_allocAndQueueBuffers(hDev, 16)
    result =  cam.cx_startAcquisition(hDev)
    if result!=base.CX_STATUS_OK:
        print("cx_startAcquisition(%d) returned error %d" % (hDev, result))

    #4. Grab image buffer, wait for image with optional timeout
    result, hBuffer = cam.cx_waitForBuffer(hDev, 1000)
    if result != base.CX_STATUS_OK:
        print("cx_waitForBuffer returned error %d" % (result))

    # 5. get image from buffer and save it
    # the img object holds a reference to the data in the internal buffer, if you need the image data after cx_queueBuffer you need to copy it!
    result, img = cam.cx_getBufferImage(hBuffer, 0)
    if result != base.CX_STATUS_OK:
        print("cx_getBufferImage returned error %d" % (result))

    #copy the image data into a numpy array
    image = np.copy(img.data)

    # 6. Queue back the buffer. From now on the img.data is not valid anymore and might be overwritten with new image data at any time!
    result = cam.cx_queueBuffer(hBuffer)
    if result != base.CX_STATUS_OK:
        print("cx_queueBuffer returned error %d" % (result))

    # 7. stop acquistion
    result = cam.cx_stopAcquisition(hDev)
    if result!=base.CX_STATUS_OK:
        print("cx_stopAcquisition returned error %d" % (result))

    # 8. cleanup
    result = cam.cx_freeBuffers(hDev)
    if result!=base.CX_STATUS_OK:
        print("cx_freeBuffers returned error %d" % (result))

    #result = cam.cx_closeDevice(hDev)
    #if result!=base.CX_STATUS_OK:
        #print("cx_closeDevice returned error %d" % (result))
    return image

def getSequence(num_img, hDev):
    """
    Snaps a sequence of images from the device
    """
    #Allocate and queue internal buffers (buffer = temporary image storage)
    result = cam.cx_allocAndQueueBuffers(hDev,16)
    
    #Start acquisition
    result = cam.cx_startAcquisition(hDev)
    if result != base.CX_STATUS_OK:
        print("Error, acquisition not started due to error")

    fig = plt.figure()
    plt.title("sequence")
    images = []
    
    for x in range(num_img):
        #hBuffer
        result, hBuffer = cam.cx_waitForBuffer(hDev,1000)

        if result != base.CX_STATUS_OK:
            print("Error, grab image buffer not started due to error")
        
        result, img = cam.cx_getBufferImage(hBuffer,0)
        if result != base.CX_STATUS_OK:
            print("Error, saving image buffer not started due to error")
        
        #scale the image and plot with matplotlib
        img_mean = np.mean(img.data)
        img_min = np.min(img.data)
        img_max = np.max(img.data)
        img_std = np.std(img.data)

        #render image
        axes_img = plt.imshow(img.data, vmin = img_mean - 3*img_std, vmax = img_mean + 3*img_std, cmap = "gray", interpolation="None", animated = True)
        images.append([axes_img])
        
        #Queue back the buffer. From now on the img.data is not valid anymore and might be overwritten with new image data at any time!
        result = cam.cx_queueBuffer(hBuffer)

    #show the sequence
    ani = animation.ArtistAnimation(fig,images,interval = 20, blit = True, repeat_delay = 1000)
    plt.show()

    #Stop acquisition
    result = cam.cx_stopAcquisition(hDev)
    if result != base.CX_STATUS_OK:
        print("Error, image acquisiton not ended due to error")

def defaultConfig(hDev):
    """
    Configure the camera in the default config, i.e reset the device.
    """
    cam.cx_setParam(hDev,"DeviceReset", 1)
    cam.cx_setParam(hDev,"ReverseY", 1)

def imageConfig(hDev):
    cam.cx_setParam(hDev,"CameraMode","Image")
    cam.cx_setParam(hDev,"PixelFormat","Mono8")
    cam.cx_setParam(hDev,"ExposureTimeAbs", 1500)
    cam.cx_setParam(hDev,"ReverseY", 1)
    cam.cx_setParam(hDev,"ReverseX", 0)

def cogConfig(hDev):
    """
    Configure the device to be used in Center Of Gravity Mode.
    Based on the .cxc-file found in:
    ...\Examples\CommonVisionBlox\VCGenICamGrabChunkReaderExample
    """
    cam.cx_setParam(hDev,"ProfileTriggerMode","FreeRun")
    cam.cx_setParam(hDev,"EnableDC2", 1)
    cam.cx_setParam(hDev,"EnableDC1", 0)
    cam.cx_setParam(hDev,"Width", 2048)
    cam.cx_setParam(hDev,"PixelFormat", "Mono16")
    cam.cx_setParam(hDev,"TestImageSelector", 1)
    cam.cx_setParam(hDev,"Output1", 1)
    cam.cx_setParam(hDev,"Output2", 0)
    cam.cx_setParam(hDev,"LaserPower", 0)
    cam.cx_setParam(hDev,"TurnLaserOn", 0)
    cam.cx_setParam(hDev,"TurnLaserOnAuto", 0)
    cam.cx_setParam(hDev,"AcquisitionMode", 0)
    cam.cx_setParam(hDev,"AcquisitionFrameCount", 1)
    cam.cx_setParam(hDev,"EnableDC0", 0)
    cam.cx_setParam(hDev,"EnableDC1", 0)
    cam.cx_setParam(hDev,"EnableDC1TrshWidth", 0)
    cam.cx_setParam(hDev,"EnableDC1Width", 0)
    cam.cx_setParam(hDev,"EnableDC1Flags", 0)
    cam.cx_setParam(hDev,"EnableDC2", 1)
    cam.cx_setParam(hDev,"EnableDC2TrshSP", 0)
    cam.cx_setParam(hDev,"EnableDC0Shift", 0)
    cam.cx_setParam(hDev,"CameraMode", 3)
    cam.cx_setParam(hDev,"ProfilesPerFrame", 1)
    cam.cx_setParam(hDev,"ClearInvalidPos",0)
    cam.cx_setParam(hDev,"PosValidationEn", 0)
    cam.cx_setParam(hDev,"AbsOffsetPos", 0)
    cam.cx_setParam(hDev,"ThrshFirstFalling", 0)
    cam.cx_setParam(hDev,"NumCOGSP", 6) #2^6 = values between 0 and 65536
    cam.cx_setParam(hDev,"ValidationWidthMin", 0)
    cam.cx_setParam(hDev,"ValidationWidthMax", 1087)
    cam.cx_setParam(hDev,"ValidationSumMin", 0)
    cam.cx_setParam(hDev,"ValidationSumMax", 65535)
    cam.cx_setParam(hDev,"GainPGA", 0)
    cam.cx_setParam(hDev,"GainADC", 44)
    cam.cx_setParam(hDev,"Vlow2", 96)
    cam.cx_setParam(hDev,"Vlow3", 96)
    cam.cx_setParam(hDev,"Vramp1", 109)
    cam.cx_setParam(hDev,"Vramp2", 109)
    cam.cx_setParam(hDev,"FOT", 10)
    cam.cx_setParam(hDev,"ExposureTimeAbs", 100)
    cam.cx_setParam(hDev,"FramePeriode", 100)
    cam.cx_setParam(hDev,"MultipleSlopeMode", 0)
    cam.cx_setParam(hDev,"NDRMode", 0)
    cam.cx_setParam(hDev,"AoiSelector", 1)
    cam.cx_setParam(hDev,"AoiHeight", 1088)
    cam.cx_setParam(hDev,"AoiOffsetY", 0)
    cam.cx_setParam(hDev,"AoiThreshold", 120)
    cam.cx_setParam(hDev,"NumAOIs", 1)
    cam.cx_setParam(hDev,"SequencerMode", 0)
    cam.cx_setParam(hDev,"ProfileTriggerMode", 0)
    cam.cx_setParam(hDev,"GevHeartbeatTimeout", 3000)
    cam.cx_setParam(hDev,"GevStreamChannelSelector", 0)
    cam.cx_setParam(hDev,"GevSCPSPacketSize", 1500)
    cam.cx_setParam(hDev,"GevSCPD", 0)
    cam.cx_setParam(hDev,"ReverseX", 0)
    cam.cx_setParam(hDev,"ReverseY", 1)
    cam.cx_setParam(hDev,"ChunkModeActive", 1)
    cam.cx_setParam(hDev,"ChunkModeSelector", 0)
    cam.cx_setParam(hDev,"EventSelector", 0)
    cam.cx_setParam(hDev,"EventNotification", 0)
    cam.cx_setParam(hDev,"EventSelector", 36882)
    cam.cx_setParam(hDev,"EventNotification", 0)
    cam.cx_setParam(hDev,"EventSelector", 36883)
    cam.cx_setParam(hDev,"EventNotification", 0)
    cam.cx_setParam(hDev,"EventSelector", 36884)
    cam.cx_setParam(hDev,"EventNotification", 0)
    cam.cx_setParam(hDev,"EventSelector", 36885)
    cam.cx_setParam(hDev,"EventSelector", 0)

#NODE FUNCTIONS
#-----------------------------------------------------------------------------------------------------------------------------------
def print_info(hDev, name):
    """
    Taken from AT's cx_cam_enumerate_nodemap.py file.
    Prints information about the node, i.e print_info(hDev,"CameraMode")
    """
    val = "None"
    sym_entries = ""

    result, type = cam.cx_getParamInfo(hDev, cam.CX_PARAM_INFO_TYPE, name)
    if result != base.CX_STATUS_OK:
        print("cx_getParamInfo(Type) from %s returned error %d" % (name, result))
        return
    result, range = cam.cx_getParamInfo(hDev, cam.CX_PARAM_INFO_RANGE, name)
    if result != base.CX_STATUS_OK:
        print("cx_getParamInfo(Range) from %s returned error %d" % (name, result))
        return
    result, descr = cam.cx_getParamInfo(hDev, cam.CX_PARAM_INFO_DESCRIPTION, name)
    if result != base.CX_STATUS_OK:
        print("cx_getParamInfo(Description) from %s returned error %d" % (name, result))
        return
    result, tooltip = cam.cx_getParamInfo(hDev, cam.CX_PARAM_INFO_TOOLTIP, name)
    if result != base.CX_STATUS_OK:
        print("cx_getParamInfo(Tooltip) from %s returned error %d" % (name, result))
        return
    result, access = cam.cx_getParamInfo(hDev, cam.CX_PARAM_INFO_ACCESSS_MODE, name)
    if result != base.CX_STATUS_OK:
        print("cx_getParamInfo(AccessMode) from %s returned error %d" % (name, result))
        return
    result, visibility = cam.cx_getParamInfo(hDev, cam.CX_PARAM_INFO_VISIBILITY, name)
    if result != base.CX_STATUS_OK:
        print("cx_getParamInfo(Visibility) from %s returned error %d" % (name, result))
        return
    if type == cam.CX_PARAM_ENUM:
        result, sym_entries = cam.cx_getParamInfo(hDev, cam.CX_PARAM_INFO_ENUM_SYMBOLS, name)
        if result != base.CX_STATUS_OK:
            print("cx_getParamInfo(Flags) from %s returned error %d" % (name, result))
            return
    if access == cam.CX_PARAM_ACCESS_RO or access == cam.CX_PARAM_ACCESS_RW:
        result, val = cam.cx_getParam(hDev, name)
        if result != base.CX_STATUS_OK:
            print("cx_getParam(Value) from %s returned error %d" % (name, result))
            return
    if type == cam.CX_PARAM_CATEGORY:
        result, cat_children = cam.cx_getParamInfo(hDev, cam.CX_PARAM_INFO_CATEGORY_CHILDS, name)
        if result != base.CX_STATUS_OK:
            print("cx_getParam(Children) from %s returned error %d" % (name, result))
            return

    visibility_str = "Undefined"
    if visibility == cam.CX_PARAM_VISIBILITY_BEGINNER:
        visibility_str = "Beginner"
    if visibility == cam.CX_PARAM_VISIBILITY_EXPERT:
        visibility_str = "Expert"
    if visibility == cam.CX_PARAM_VISIBILITY_GURU:
        visibility_str = "Guru"
    if visibility == cam.CX_PARAM_VISIBILITY_INVISIBLE:
        visibility_str = "Invisible"

    access_str = ""
    if access == cam.CX_PARAM_ACCESS_RO:
        access_str="RO"
    elif access == cam.CX_PARAM_ACCESS_WO:
        access_str="WO"
    elif access == cam.CX_PARAM_ACCESS_RW:
        access_str="RW"
    elif access == cam.CX_PARAM_ACCESS_NOT_AVAILABLE:
        access_str="Not Available"
    elif access & cam.CX_PARAM_ACCESS_NOT_IMPLEMENTED:
        access_str="Not Implemented"

    type_str = "Unknown"
    if type == cam.CX_PARAM_INTEGER:
        type_str = "Integer"
    elif type == cam.CX_PARAM_FLOAT:
        type_str = "Float"
    elif type == cam.CX_PARAM_STRING:
        type_str = "String"
    elif type == cam.CX_PARAM_ENUM:
        type_str = "Enum"
    elif type == cam.CX_PARAM_BOOLEAN:
        type_str = "Boolean"
    elif type == cam.CX_PARAM_COMMAND:
        type_str = "Command"
    elif type == cam.CX_PARAM_CATEGORY:
        type_str = "Category"

    print("Information for %s" % (name))
    print("\tType: %s" % (type_str))
    print("\tDescription: %s"  % (descr))
    print("\tTooltip: %s" % (tooltip))
    print("\tAccess: %s" % (access_str))
    print("\tVisibility: %s" % (visibility_str))
    print("\tValue: %s" % (str(val)))

    if type == cam.CX_PARAM_ENUM:
        sym_list = tuple(sym_entries.split('\0')) # shows how to convert to a list...
        print("\tEntries: %s" % ', '.join(map(str, sym_list)))
        print("\tRange: %s" % ', '.join([str(a) for a in range]))
    elif type == cam.CX_PARAM_STRING:
        print("\tRange: %s" % (str(range)))
    elif type == cam.CX_PARAM_COMMAND:
        print("\tRange: %s" % (str(range)))
    elif type == cam.CX_PARAM_INTEGER:
        print("\tRange: %s..%s" % (str(range[0]), str(range[1])))
    elif type == cam.CX_PARAM_CATEGORY:
        child_list = tuple(cat_children.split('\0'))  # shows how to convert to a list...
        print("\tChildren: %s" % ', '.join(map(str, child_list)))

def checkStatusCode(code):
    """
    Outputs the status code in text format
    Can be helpful in debugging
    """
    if isinstance(code, int):
        if code == 0:
            print("Status is: OK")
        elif code == -1:
            print("Status is: Failed")
        elif code == -2:
            print("Status is: Not Implemented")
        elif code == -3:
            print("Status is: Open Failed")
        elif code == -4:
            print("Status is: Device Not Open")
        elif code == -5:
            print("Status is: Out Of Memory")
        elif code == -6:
            print("Status is: Timeout")
        elif code == -11:
            print("Status is: Invalid Parameter")
        elif code == -13:
            print("Status is: Buffer Too Small")
        elif code == -14:
            print("Status is: Device Already Open")
        elif code == -15:
            print("Status is: Access Denied")
        elif code == -16:
            print("Status is: Device Busy")
        elif code == -18:
            print("Status is: No Data")
        elif code == -19:
            print("Status is: Invalid Handle")
        elif code == -20:
            print("Status is: Unknown Parameter")
        elif code == -21:
            print("Status is: Bad Format")
        elif code == -22:
            print("Status is: Not Supported")
        elif code == -24:
            print("Status is: Already registrered")

#OTHER
#-----------------------------------------------------------------------------------------------------------------------------------
def sortList(unsortedList):
    #sort a list in alphanumeric order
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)',key)]
    return sorted(unsortedList,key = alphanum_key)

def Img3STD(img):
    img_mean = np.mean(img.data)
    img_min = np.min(img.data)
    img_max = np.max(img.data)
    img_std = np.std(img.data)

    return img_mean, img_min, img_max, img_std

def getCurrentWD():
    """
    Get path to current work directory with correct backslash format
    used in glob.
    """
    wd = os.getcwd().replace("\\", "/")
    return wd

class Chessboard:
    def __init__(self):
        #chessboard dimensions
        self.cbrow = 6
        self.cbcol = 10
        self.sqrsize = 30 #mm
        #termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, self.sqrsize, 0.001)
        self.objp = np.zeros((self.cbrow*self.cbcol,3),np.float32)
        self.objp[:,:2] = np.mgrid[0:self.cbcol,0:self.cbrow].T.reshape(-1,2)

def print_chunk(timeStamp64L,timeStamp64H,frameCnt,triggerCoord,triggerStatus,AO0,AI0,INT_idx,AOI_idx,AOI_ys,AOI_dy,AOI_xs,AOI_trsh,AOI_alg):
    """
    Prints out "ChunckAcqInfo" in a readable format
    """
    #64-bit timestamp
    print("\nTimeStamp64L: " + str(timeStamp64L))
    print("TimeStamp64H: " + str(timeStamp64H))
    #32-bit frame counter
    print("Framecount: " + str(frameCnt))
    #32-bit trigger coordinate
    print("TriggerCoordinate: " + str(triggerCoord))
    #Trigger status
    print("TriggerStatus: " + str(triggerStatus))
    #I/0 status
    print("AO0: " + str(AO0))
    print("AO1: " + str(AI0))

    print("INT_idx: " + str(INT_idx))
    print("AOI_idx: " + str(AOI_idx))
    print("AOI_ys: " + str(AOI_ys))
    print("AOI_dy: " + str(AOI_dy))
    print("AOI_xs: " + str(AOI_xs))
    print("AOI_trsh: " + str(AOI_trsh))
    print("AOI_alg: " + str(AOI_alg))
    
def pixCoordify(laser,width):
    """
    This function reshapes the COG-image from (2048,) to (2048,2). 
    This new array contains both the x and y-values of the laser image.
    """
    liste = list(range(width))
    liste = np.vstack((liste,laser)).T
    return liste

def loadCaliParam():
    """
    This functions loads the camera calibration parameters
    obtained from Matlab into numpy arrays
    """
    #path to the folder where the parameters are saved
    
    #Old calibration files
    caliParam_folder = "C:/Users/trymdh.WIN-NTNU-NO/OneDrive/tpk4940Master/Matlab" #work pc
    #caliParam_folder = "C:/Users/Trym/OneDrive/tpk4940Master/Matlab" # home pc
    #caliParam_folder = "C:/Users/TrymAsus/OneDrive/tpk4940Master/Matlab" #LAPTOP
    
    #New calibration files:
    #caliParam_folder = "C:/Users/trymdh.WIN-NTNU-NO/OneDrive/tpk4940Master/Espen Code/Matlab"
    #caliParam_folder = "C:/Users/TrymAsus/OneDrive/tpk4940Master/Espen Code/Matlab" #LAPTOP
    #caliParam_folder = "C:/Users/Trym/OneDrive/tpk4940Master/Espen Code/Matlab" # home pc
    os.chdir(caliParam_folder)

    #Mean Reprojection Error
    ret = np.loadtxt('MeanReprojectionError.txt')
   
    #The Intrisinc Matrix
    mtx = np.loadtxt('IntrinsicMatrix.txt') #Old
    #mtx = np.loadtxt('./CalibrationConstants/calibratedCameraMatrix.txt') #New
    
    #Rotation Matrices and translation vectors between the scene and the camera
    tvecs = np.loadtxt('TranslationVectors.txt')
    rMats = np.loadtxt('RotationMatrices.txt') # Note: this file contains all the scene/camera rotationmatrices for each picture. It needs to be reshaped from (#,3) into (#/3,3,3)
    shp = rMats.shape
    C = int(shp[0]/3)
    rMats = rMats.reshape(C,3,3)
    
    
    #Radial and tangential distortion coeffecients, dist = [k_1,k_2,p_1,p_2[,k_3[,k_4,k_5,k_6]]]
    dist = []
    #rDist = np.loadtxt('./CalibrationConstants/calibratedRaddist.txt') #k_1 and k_2, => k_3 = 0, this leads to dist = [k_1,k_2,p_1,p_2]
    rDist = np.loadtxt('RadialDistortion.txt')
    #tDist = np.loadtxt('./CalibrationConstants/calibratedTangdist.txt') #p_1 and p_2
    tDist = np.loadtxt('TangentialDistortion.txt') #p_1 and p_2
    
    dist.append(rDist)
    dist.append(tDist)
    dist = np.asarray(dist).reshape(1,4)

    return ret,mtx,tvecs,rMats,dist

def extractPoints(laser_npy,rMats,tvecs,K,distCoeff):
    K_inv = np.linalg.inv(K)
    ext_points = np.array([])
    j = 0
    for i in range(1,len(laser_npy) + 1):
        RotM = rMats[j]
        tVec = tvecs[j]
        T = np.eye(4)
        T[0:3,0:3] = RotM
        T[0:3,3] = tVec
        n = RotM[2,:]
        p_0 = tVec
        l_0 = np.array([0,0,0])
        fname = 'pixcoord_' + str(i) + '.npy'
        pix_coord = np.load(fname)

        for coord in pix_coord:
            if coord[1] != 0:
                coord = coord.reshape(-2,1,2)
                coord = cv2.undistortPoints(coord,K,distCoeff).reshape(-1,2)
                coord = np.append(coord,1)
                img_coord = K_inv@coord
                norm_img_coord = img_coord/img_coord[2]
                l = norm_img_coord
                num = np.dot((p_0 - l_0), n)
                deno = np.dot(l,n)
                d = num/deno
                fullcoord = np.array([norm_img_coord * d]) + l_0
                ext_points = np.append(ext_points,fullcoord)
        j = j + 1
    ext_points = np.reshape(ext_points,(-1,3))
    return ext_points

def drawFrame(R,t):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim((-0.5,0.5))
    ax.set_ylim((-1,0))
    ax.set_zlim(-0.5,0.5)
    
    i = 0
    for r in R:
        origin = [t[i][0],t[i][1],t[i][2]]
        x = r[0:3,0]
        y = r[0:3,1]
        z = r[0:3,2]
        f = 10
        x_ax = ax.quiver(origin[0],origin[1],origin[2],x[0]/f,x[1]/f,x[2]/f, color = 'red', arrow_length_ratio = 0.05)
        y_ax = ax.quiver(origin[0],origin[1],origin[2],y[0]/f,y[1]/f,y[2]/f, color = 'blue', arrow_length_ratio = 0.05)
        z_ax = ax.quiver(origin[0],origin[1],origin[2],z[0]/f,z[1]/f,z[2]/f, color = 'green',arrow_length_ratio = 0.05)
        i = i + 1
    plt.show()

    return 0

def getCentroid3D(pointCloud):
    #this function returns the centroid
    xs = pointCloud[:,0]; ys = pointCloud[:,1]; zs = pointCloud[:,2]
    x_av = np.average(xs); y_av = np.average(ys); z_av = np.average(zs)

    return np.array([x_av,y_av,z_av])

def getPlaneData(pI,ax):
    #This function takes the plane parameters and the matplotlib plot object as input 
    #and returns the data values needed to plot a wireframe using plt.subplot.plot_wireframe()
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),np.arange(ylim[0], ylim[1]))
    Z = np.zeros(X.shape)
    for r in range(X.shape[0]):
        for c in range(X.shape[1]):
            #z = a*X + b*Y + d 
            Z[r,c] = -(pI[0] * X[r,c] + pI[1] * Y[r,c] + pI[3])*1./pI[2]

    return X,Y,Z

def estimatePlane(points):
    #This function estimates a plane from three points and return the plane parameters
    #if the condition for point to be in the plane is satisfied.
    p_1 = points[0]; p_2 = points[1]; p_3 = points[2]
    #compute two vectors in the plane
    v1 = p_1 - p_3; v2 = p_2 - p_3
    #centroid
    cent = getCentroid3D(points)
    #The plane normal is then the cross product of these two vectors, n = [a,b,c]
    n = np.cross(v1,v2)
    #distance from the plane to the origin
    d = -np.dot(p_3,np.cross(p_1,p_2))*np.linalg.norm(n)
    #Plane, pI = [n,d] = [a,b,c,d]
    pI = np.append(n,d)
    #p_h = [x,y,z,1]
    p_1_h = np.append(p_1,1); p_2_h = np.append(p_2,1); p_3_h = np.append(p_3,1)
    #Criteria for a point to be in the plane is x_h . Pi ~ 0 is satisfied.
    cri1 = np.dot(p_1_h,pI); cri2 = np.dot(p_2_h,pI); cri3 = np.dot(p_3_h,pI)
    conditions = [np.around(cri1,decimals=7) == 0,np.around(cri3,decimals=7) == 0,np.around(cri3,decimals=7) == 0]

    if all(conditions):
        return pI,cent
    else:
        return None

def error_checker(plane,point_cloud):
    [A,B,C,D] = plane
    distances = np.array([])
    for i in range(len(point_cloud[:,0])):
        [x,y,z] = [point_cloud[i,0],point_cloud[i,1],point_cloud[i,2]] 
        d = abs(A*x + B*y + C*z + D)/np.sqrt(A**2 + B**2 + C**2)
        distances = np.append(distances,d)
    return sum(distances)/len(distances)

def plotError(error_vec,median_error,error_std):
    plt.ion
    plt.figure(1)
    plt.hist(error_vec,bins=np.linspace(-4,4,50))
    plt.show()

def plotError2(error_vec,median_error,error_std):
    stand_err = error_vec[np.abs(error_vec) < 4]
    mean = np.mean(stand_err)
    std = np.std(stand_err)
    stand_err = (stand_err-mean)/std
    print("std:{0} and mean: {1}".format(std,mean))
    plt.ion
    plt.figure(1)
    plt.hist(stand_err,bins=np.linspace(-4,4,1000))
    plt.show()
    
def getError(pointCloud,pI,c):
    [A,B,C,D] = pI
    error_list = []
    for point in pointCloud:
        d = (A*(point[0]-c[0]) + B*(point[1]-c[1]) + C*(point[2]-c[2]))/np.sqrt(A**2 + B**2 + C**2)
        error_list.append(d)
    error_vec = np.array(error_list)
    median_error = np.median(error_vec)
    error_std = np.std(error_vec)
    #error_vec contains distances between each point in pointCloud and the plane pI
    return error_vec,median_error,error_std

def ransacPlane(pointCloud):
    bestFit = None
    bestErr = np.inf
    centroid = None
    best_cnt_in = 0
    goal_inlier = 0.7*len(pointCloud)
    N = np.inf
    ite = 0
    while ite < N:
        #sample 3 random points and estimate plane
        maybeIndex = np.random.choice(pointCloud.shape[0],3,replace = False)
        maybeInliers = pointCloud[maybeIndex,:]
        pI,c = svd_AxB(homogenify(maybeInliers))
        
        #calculate error and inlier threshold
        error_vec,median_error,error_std = getError(pointCloud,pI,c)
        #count inliers
        Inliers = countInliers(error_vec, median_error,error_std,pointCloud)
        cnt_in = len(Inliers)
        if cnt_in >= goal_inlier:
            betterData = Inliers
            #The new dataset contains few outliers => use LS to estimate plane
            #betterFit,betterRes = lsPlane(betterData)
            betterFit,c = svd_AxB(homogenify(betterData))
            error = error_checker(betterFit,pointCloud)
            if (cnt_in >= best_cnt_in) and (error < bestErr):
                #update N
                w = cnt_in/len(pointCloud)
                N = np.log(1-0.99)/np.log(1-w**3)
                #Update best fit plane
                bestFit = betterFit
                best_cnt_in = cnt_in
                bestErr = error
                centroid = c
                #print("\nIteration {0}".format(ite + 1))
                #print("Inlier count: {0} / {1}".format(best_cnt_in,len(pointCloud)))
                #print ("{0}x + {1}y + {2}z + {3}".format(bestFit[0], bestFit[1], bestFit[2],bestFit[3]))
                #print ("Error:")
                #print(bestErr)
        
        ite += 1
    #print("Number of iterations:{0}".format(ite + 1))
    return bestFit,centroid,bestErr

def ransacXn(pointCloud,n):
    #Running the ransac function n-times and return the best fit if the error is smaller than the "best-fit-to-date" error.
    os.chdir('C:/Users/trymdh.WIN-NTNU-NO/OneDrive/tpk4940Master/Python/VS Code')
    bestFit = np.load('BestRansacPlane.npy')
    bestPlane,bestPlane_s = planeify(bestFit)
    bestErr = error_checker(bestPlane,pointCloud)
    for i in range(0,n):
        if msvcrt.kbhit():
            key = str(msvcrt.getch()).replace("b'","").replace("'","")
            if key == 'q':
                print("Loop exited")
                break
        ransac_fit,c,err = ransacPlane(pointCloud)
        print("Current error is:{0} \t Best error is: {1}".format(err,bestErr))
        if err < bestErr:
            bestFit = np.append(ransac_fit[0:3],c)
            bestPlane,bestPlane_s = planeify(bestFit)
            bestErr = err
            print("BestError: {0}".format(bestErr))
            np.save('C:/Users/trymdh.WIN-NTNU-NO/OneDrive/tpk4940Master/Python/VS Code/BestRansacPlane.npy',bestFit)
    return bestPlane,c,bestErr

def homogenify(G):
    H = []
    for point in G:
        H.append(np.append(point,1))
    return np.asarray(H)

def svd_AxB(pointCloud):
    u,s,vh = np.linalg.svd(pointCloud)
    v = vh.conj().T
    x = v[:,-1]
    x = x.T.reshape(-1)
    c = getCentroid3D(pointCloud)
    n = x[0:3]
    d = x[3]/np.linalg.norm(n)
    #[A,B,C,D]
    pI = x*d
    return pI,c

def countInliers(error_vec, median_error,error_std,pointCloud):
    i = 0
    cnt_in = 0
    Inliers = []
    for error in error_vec:
        if np.abs(error) < np.abs(median_error) + 0.5*np.abs(error_std):
        #if np.abs(error) < 0.2:
            cnt_in += 1
            Inliers.append(pointCloud[i])
        i += 1
    Inliers = np.array(Inliers)
    return Inliers

def lsPlane(pointCloud):
    """
    Code below is based on the user "BEN" from 
    https://stackoverflow.com/questions/1400213/3d-least-squares-plane
    """
    # A * x_fit = b => 
    # (ax + by + d = -z)
    # where A = [[x1 y1 1], 
    #            [x2 y2 1],
    #               ...   ,
    #            [xn yn 1]]
    # b = [-z1, -z2, ... , -zn].T
    # and x_fit = [a b d]

    x = pointCloud[:,0]
    y = pointCloud[:,1]
    z = pointCloud[:,2]
    c = getCentroid3D(pointCloud)

    A = []
    b = []
    for i in range(len(x)):
        A.append([x[i], y[i], 1])
        b.append(z[i])
    A = np.matrix(A)
    b = np.matrix(b).T

    fit = (A.T @ A).I @ A.T @ b 
    #error = vertical offset between point and plane
    #error = z_i - z_proj
    error = b - A @ fit

    #planenormal is then
    v1 = np.array([1,0,fit[0]])
    v2 = np.array([0,1,fit[1]])
    
    n = np.cross(v1,v2)
    n /= np.linalg.norm(n)

    return np.append(n,c)

#Matrix functions
#--------------------------------------------------------------------------------------------------------------------------------
def skew(k):
    return np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])

def unskew(SS):
    """
    This function takes in a skew symmetrix matrix and returns
    it on vector form.
    """
    x = SS[2,1]
    y = SS[0,2]
    z = SS[1,0]
    return np.array([[x,y,z]]).T

def logMatrix(R):
    """
    A = |R(theta) t(x,y,z)|
        |   0        1    |
    when |theta| < pi:
        tr(R) = 1 + 2*cos(theta)
    ----------------------------------------
    log A = (R-R.T)*(theta/2*sin(theta))
    """
    theta = np.arccos((np.trace(R)-1)/2)
    #print(np.linalg.norm(theta) < np.pi)
    
    log_A_skewsym = (R-R.T)*theta/(2*np.sin(theta))
    log_A = unskew(log_A_skewsym)
    return log_A

#ESPEN CODE
def planeify(vector_plane): #Assumes form [a,b,c,x_0,y_0,z_0]
    #a*x_0 + b*y_0 + c*z_0 + d = 0
    #-> d = - (a*x_0 + b*y_0 + c*z_0)
    D = -(vector_plane[0]*(vector_plane[3])+vector_plane[1]*(vector_plane[4])+vector_plane[2]*(vector_plane[5]))
    #[A,B,C,D]
    plane = np.asarray([vector_plane[0],vector_plane[1],vector_plane[2],D])
    #Or on form [Ax + By + D] = z
    plane_s = np.asarray([-plane[0]/plane[2],-plane[1]/plane[2],-plane[3]/plane[2]])
    return plane,plane_s

def error_checker(plane,point_cloud):
    [A,B,C,D] = plane
    distances = np.array([])
    for i in range(len(point_cloud[:,0])):
        [x,y,z] = [point_cloud[i,0],point_cloud[i,1],point_cloud[i,2]] 
        d = abs(A*x + B*y + C*z + D)/np.sqrt(A**2 + B**2 + C**2)
        distances = np.append(distances,d)
        
    return sum(distances)/len(distances)