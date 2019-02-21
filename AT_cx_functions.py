import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import cx.cx_cam as cam
import cx.cx_base as base

#NODE FUNCTIONS
#-----------------------------------------------------------------------------------------------------------------------------------
#prints node information, input e.g: print_info(hDev,"CameraMode")
def print_info(hDev, name):
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

def cog_cam_config(hDev):
    #setting the cameraMode to COG
    cam.cx_setParam(hDev,"CameraMode","CenterOfGravity")

    #Other config...

def default_cam_config(hDev):
    #resetting the device to default
    cam.cx_setParam(hDev,"DeviceReset",True)

#-----------------------------------------------------------------------------------------------------------------------------------
#Snap an image sequence from the device
def getSequence(num_img, hDev):
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

def getDevice():
    #find connected devices and assign to device handle
    #hDev = device handle
    #result is set to base.CX_STATUS_OK if succesful.
    #base.CX_STATUS_OK = 0
    result = cam.cx_dd_findDevices("", 2000, cam.CX_DD_USE_GEV | cam.CX_DD_USE_GEV_BROADCAST)
    
    #Number of connected devices
    result, num_dev = cam.cx_dd_getNumFoundDevices()
    if result != base.CX_STATUS_OK:
        print("Error, no cameras found")

    #if device is found, get device uri
    result, uri = cam.cx_dd_getParam(0,"URI")
    print(str(num_dev) + " camera found\nURI:\n"+ uri)    
    #connect device URI of the first discovered device
    result, hDev = cam.cx_openDevice(uri)

    return hDev