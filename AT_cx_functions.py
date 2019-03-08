import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import cx.cx_cam as cam
import cx.cx_base as base
import cv2
import os
import time

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
        self.cbrow = 7
        self.cbcol = 9
        self.sqrsize = 20 #mm
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
