import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
#import cx.cx_cam as cam
#import cx.cx_base as base
import cv2
import os
import time
import random

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
    #caliParam_folder = "C:/Users/trymdh.WIN-NTNU-NO/OneDrive/tpk4940Master/Matlab" #work pc
    caliParam_folder = "C:/Users/Trym/OneDrive/tpk4940Master/Matlab" # home pc
    #caliParam_folder = "C:/Users/TrymAsus/OneDrive/tpk4940Master/Matlab" #LAPTOP

    os.chdir(caliParam_folder)
    
    #Mean Reprojection Error
    ret = np.loadtxt('MeanReprojectionError.txt')
   
    #The Intrisinc Matrix
    mtx = np.loadtxt('IntrinsicMatrix.txt')
    
    #Rotation Matrices and translation vectors between the scene and the camera
    tvecs = np.loadtxt('TranslationVectors.txt')
    rMats = np.loadtxt('RotationMatrices.txt') # Note: this file contains all the scene/camera rotationmatrices for each picture. It needs to be reshaped from (#,3) into (#/3,3,3)
    shp = rMats.shape
    C = int(shp[0]/3)
    rMats = rMats.reshape(C,3,3)
    
    #Radial and tangential distortion coeffecients, dist = [k_1,k_2,p_1,p_2[,k_3[,k_4,k_5,k_6]]]
    dist = []
    rDist = np.loadtxt('RadialDistortion.txt') #k_1 and k_2, => k_3 = 0, this leads to dist = [k_1,k_2,p_1,p_2]
    tDist = np.loadtxt('TangentialDistortion.txt') #p_1 and p_2
    dist.append(rDist)
    dist.append(tDist)
    dist = np.asarray(dist).reshape(1,4)

    return ret,mtx,tvecs,rMats,dist

def extractPoints(laser_npy,rMats,tvecs,K_inv):
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
        filename = 'pixcoord_' + str(i) + '.npy'
        pix_coord = np.load(filename)
        for coord in pix_coord:
            if coord[1] != 0:
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
    #this function takes in all the points seperated into their x,y and z coordinates, 
    #and returns the centroid of all the points.
    xs = pointCloud[:,0]
    ys = pointCloud[:,1]
    zs = pointCloud[:,2]

    C = []
    x_av = np.average(xs)
    y_av = np.average(ys)
    z_av = np.average(zs)
    C.append(x_av)
    C.append(y_av)
    C.append(z_av)
    return C

def getPlaneData(pI,ax,ls = False,rnsc = False):
    #This function takes the plane parameters and the matplotlib plot object as input 
    #and returns the data values needed to plot a wireframe using plt.subplot.plot_wireframe()
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),
                    np.arange(ylim[0], ylim[1]))
    Z = np.zeros(X.shape)
    if ls:
        for r in range(X.shape[0]):
            for c in range(X.shape[1]):
                #z = a*X + b*Y + d    
                Z[r,c] = pI[0] * X[r,c] + pI[1] * Y[r,c] + pI[3]

    elif rnsc:
        for r in range(X.shape[0]):
            for c in range(X.shape[1]):
                #z = a*X + b*Y + d    
                Z[r,c] = -(pI[0] * X[r,c] + pI[1] * Y[r,c] + pI[3])*1./pI[2]

    return X,Y,Z

def estimatePlane(P_rand):
    #This function estimates a plane from three random points taken from a dataset and return the plane parameters
    #if the condition for point to be in the plane is satisfied.
    p_1 = P_rand[0]
    p_2 = P_rand[1]
    p_3 = P_rand[2]
    
    #compute two vectors in the plane
    v1 = p_1 - p_3
    v2 = p_2 - p_3

    #The plane normal is then the cross product of these two vectors, n = [a,b,c]
    n = np.cross(v1,v2)
    
    #distance from the plane to the origin
    d = -np.dot(p_3,np.cross(p_1,p_2))
    
    #Plane, pI = [n,d] = [a,b,c,d]
    pI = np.append(n,d)
    
    #Criteria for a point to be in the plane is x_h . Pi ~ 0 is satisfied.
    #p_h = [x,y,z,1]
    p_1_h = np.append(p_1,1)
    p_2_h = np.append(p_2,1)
    p_3_h = np.append(p_3,1)
    
    c1 = np.dot(p_1_h,pI)
    c2 = np.dot(p_2_h,pI)
    c3 = np.dot(p_3_h,pI)
    
    conditions = [np.around(c1,decimals=7) == 0,
    np.around(c3,decimals=7) == 0,
    np.around(c3,decimals=7) == 0]

    if all(conditions):
        #print("The plane equation is {0}x + {1}y + {2}z = {3}".format(pI[0],pI[1],pI[2],pI[3]))
        return pI
    else:
        return None

def ransacPlane(pointCloud):
    ite = 0
    bestFit = None
    bestError = np.inf
    centroid = None
    k = 2000
    t = 1 #mm
    goal_inliers = 0.5*len(pointCloud)
    while ite < k:
        maybeIndex = np.random.choice(pointCloud.shape[0],3,replace = False)
        maybeInliers = pointCloud[maybeIndex,:]
        maybeModel = svd_AxB(maybeInliers)
        n = maybeModel[0:3]
        c = getCentroid3D(maybeInliers)
        alsoInliers = []
        cnt_in = 0
        for point in pointCloud:
            if point not in maybeInliers:
                point_h = np.append(point,1)
                error = np.linalg.norm(point_h@maybeModel)
                if np.abs(error) < t:
                    alsoInliers.append(point)
                    cnt_in = cnt_in + 1
        if cnt_in > goal_inliers:
            betterData = np.vstack([alsoInliers, maybeInliers])
            betterModel = svd_AxB(betterData)
            betterData = homogenify(betterData)
            thisErr = np.linalg.norm(betterData@betterModel)
            if thisErr < bestError:
                bestFit = betterModel
                bestError = thisErr
                centroid = getCentroid3D(betterData)
                print(bestError)
                print(cnt_in)
        ite = ite + 1
    return bestFit,centroid,bestError

def homogenify(G):
    H = []
    for point in G:
        H.append(np.append(point,1))
    return np.asarray(H)

def svd_AxB(pointCloud):
    """
    C = getCentroid3D(pointCloud)
    A = []
    for point in pointCloud:
        A.append(point-C)
    A = np.asarray(A)
    """
    if pointCloud.shape[1] == 3:
        A = homogenify(pointCloud)
    
    u,s,vh = np.linalg.svd(A)
    v = vh.conj().transpose()
    x = v[:,-1]
    return x

def lsPlane(pointCloud,print_out = False):
    """
    Code below is based on the user "BEN" from 
    https://stackoverflow.com/questions/1400213/3d-least-squares-plane
    """
    # A * x_fit = b => 
    # (ax + by + d = z)
    # where A = [[x1 y1 1], 
    #            [x2 y2 1],
    #               ...   ,
    #            [xn yn 1]]
    # b = [z1, z2, ... , zn].T
    # and x_fit = [a b d]

    x = pointCloud[:,0]
    y = pointCloud[:,1]
    z = pointCloud[:,2]

    A = []
    b = []
    for i in range(len(x)):
        A.append([x[i], y[i], 1])
        b.append(z[i])
    A = np.matrix(A)
    b = np.matrix(b).T

    #A*fit = B 
    # => (A' * A) * fit = A' * B
    # => fit = (A' * A)^(-1) * A' * B
    #fit = (A' * A)^(-1) * (A' * B)
    fit = (A.T @ A).I @ A.T @ b 

    #error = vertical offset between point and plane
    error = b - A @ fit
    
    #Frobenius norm
    residual = np.linalg.norm(error) 
    
    if print_out:
        print ("LS solution:")
        #fit[0] = a, fit[1] = b, fit[2] = d, c = 1
        print ("%f x + %f y + %f = z" % (fit[0], fit[1], fit[2]))
        print ("residual:")
        print(residual)
    
    #reshape fit into ax + by + cz + d     
    fit = np.insert(fit,2,1,axis = 0)
    return fit,residual
         
#Matrix functions
#--------------------------------------------------------------------------------------------------------------------------------
def skew(k):
    return np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])













        
    

