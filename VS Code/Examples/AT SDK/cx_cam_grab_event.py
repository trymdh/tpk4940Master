######################################################################################
## @package : CxCamLib
#  @file : cx_cam_grab_event.py
#  @brief Python example of using AT Camera library.
#  @copyright (c) 2017, Automation Technology GmbH
#  @version 07.11.2017, AT: initial version
######################################################################################

import numpy as np
from matplotlib import pyplot as plt
import cx.cx_cam as cam
import cx.cx_base as base

ACQUISITION_START_NODE = "AcquisitionStart"
EVENT_ACQUISITION_START_ID = "9012"
CX_CAM_EVENT_ACQUISITION_START = "EventAcquisitionStart"
CX_CAM_EVENT_CONNECTION_LOST = "EventConnectionLost"
CX_CAM_EVENT_CONNECTION_RESTORED = "EventConnectionRestored"
CX_CAM_EVENT_DATA_TIMESTAMP = "Timestamp"

# evnet handler for device events
def OnDeviceEvent(hDev, name, hData):
    print("DeviceEvent: %s" % (name) )

#event handler for gev and node events
def OnEvent(hDev, name, hData):
    if name == EVENT_ACQUISITION_START_ID or name == CX_CAM_EVENT_ACQUISITION_START:
        # handle gev event
        result, ts = cam.cx_getEventData(hDev, hData, CX_CAM_EVENT_DATA_TIMESTAMP)
        if result == base.CX_STATUS_OK:
            print("\nGevEvent: %s" % (EVENT_ACQUISITION_START_ID) )
            print("%s: %d" % (CX_CAM_EVENT_DATA_TIMESTAMP, ts) )
    elif name == ACQUISITION_START_NODE:
        print("\nNodeEvent: %s" %(ACQUISITION_START_NODE))


# discover cameras, you can also filter for discovering only certain cameras, e.g. "filter://169.254.213.222"
result = cam.cx_dd_findDevices("", 2000, cam.CX_DD_USE_GEV | cam.CX_DD_USE_GEV_BROADCAST)
if result!=base.CX_STATUS_OK:
    print("cx_dd_findDevices returned error %d" % (result))
    exit(0)

# get device URI of first discovered device
result, uri = cam.cx_dd_getParam(0, "URI")

# connect the camera with given URI, you can also open the camera with a valid URI without the discovering step above
# e.g. uri = "gev://169.254.239.221/?mac=00-50-C2-8E-DD-EE"
result, hDev = cam.cx_openDevice(uri)
if result!=base.CX_STATUS_OK:
    print("cx_openDevice(%s) returned error %d" % (uri, result))
    exit(-1)

# enable "AcquisitionStart" event
sval = "AcquisitionStart"   # select "AcquisitionStart" event
result = cam.cx_setParam(hDev, "EventSelector", sval)
if result!=base.CX_STATUS_OK:
    print("cx_setParam(EventSelector, %s) returned error %d" % (sval, result))
    exit(-1)
cam.cx_setParam(hDev, "EventNotification", 1)

# register the device event connection lost
resut, hConnectionLostEvent = cam.cx_registerEvent(hDev, CX_CAM_EVENT_CONNECTION_LOST, OnDeviceEvent)
if result!=base.CX_STATUS_OK:
    print("cx_registerEvent(hDev, CX_CAM_EVENT_CONNECTION_LOST, OnDeviceEvent) returned error %d" % (result))

# register the device event connection restored
resut, hConnectionRestoredEvent = cam.cx_registerEvent(hDev, CX_CAM_EVENT_CONNECTION_RESTORED, OnDeviceEvent)
if result!=base.CX_STATUS_OK:
    print("cx_registerEvent(hDev, CX_CAM_EVENT_CONNECTION_RESTORED, OnDeviceEvent) returned error %d" % (result))

# register gev event callback by name
resut, hAcquisitionStartEvent = cam.cx_registerEvent(hDev, CX_CAM_EVENT_ACQUISITION_START, OnEvent)
if result!=base.CX_STATUS_OK:
    print("cx_registerEvent(hDev, CX_CAM_EVENT_ACQUISITION_START, OnEvent) returned error %d" % (result))

# register gev event callback by id
resut, hAcquisitionStartIdEvent = cam.cx_registerEvent(hDev, EVENT_ACQUISITION_START_ID, OnEvent)
if result!=base.CX_STATUS_OK:
    print("cx_registerEvent(hDev, EVENT_ACQUISITION_START_ID, OnEvent) returned error %d" % (result))

# register node event callback
resut, hAcquistionStartNodeEvnet = cam.cx_registerEvent(hDev, ACQUISITION_START_NODE, OnEvent)
if result!=base.CX_STATUS_OK:
    print("cx_registerEvent(hDev, ACQUISITION_START_NODE, OnEvent) returned error %d" % (result))

# allocate and queue internal buffers
result =  cam.cx_allocAndQueueBuffers(hDev, 16)
if result!=base.CX_STATUS_OK:
    print("cx_allocAndQueueBuffers(%d) returned error %d" % (hDev, result))

# start image acquistion
result =  cam.cx_startAcquisition(hDev)
if result!=base.CX_STATUS_OK:
    print("cx_startAcquisition(%d) returned error %d" % (hDev, result))

# Grab image buffer, wait for image with optional timeout
result, hBuffer = cam.cx_waitForBuffer(hDev, 10000)
if result != base.CX_STATUS_OK:
    print("cx_waitForBuffer returned error %d" % (result))

# get image from buffer
	# the img object holds a reference to the data in the internal buffer, if you need the image data after cx_queueBuffer you need to copy it!
result, img = cam.cx_getBufferImage(hBuffer, 0)
if result != base.CX_STATUS_OK:
    print("cx_getBufferImage returned error %d" % (result))
else:
    # matplotlib image display, scaled with 3*std
    img_mean = np.mean(img.data)
    img_min = np.min(img.data)
    img_max = np.max(img.data)
    img_std = np.std(img.data)

    plt.title("Range Image")
    axes_img = plt.imshow(img.data, vmin=img_mean - 3 * img_std, vmax=img_mean + 3 * img_std, cmap='gray',
                          interpolation='None', animated=True)
    plt.colorbar()
    plt.show()

# Queue back the buffer. From now on the img.data is not valid anymore and might be overwritten with new image data at any time!
result = cam.cx_queueBuffer(hBuffer)
if result != base.CX_STATUS_OK:
    print("cx_queueBuffer returned error %d" % (result))

# stop acquistion
result = cam.cx_stopAcquisition(hDev)
if result!=base.CX_STATUS_OK:
    print("cx_stopAcquisition returned error %d" % (result))

#cleanup

# unregister events

result = cam.cx_unregisterEvent(hDev, hConnectionLostEvent)
result = cam.cx_unregisterEvent(hDev, hConnectionRestoredEvent)
result = cam.cx_unregisterEvent(hDev, hAcquisitionStartEvent)
result = cam.cx_unregisterEvent(hDev, hAcquisitionStartIdEvent)
result = cam.cx_unregisterEvent(hDev, hAcquistionStartNodeEvnet)

result = cam.cx_freeBuffers(hDev)
if result!=base.CX_STATUS_OK:
    print("cx_freeBuffers returned error %d" % (result))

result = cam.cx_closeDevice(hDev)
if result!=base.CX_STATUS_OK:
    print("cx_closeDevice returned error %d" % (result))
