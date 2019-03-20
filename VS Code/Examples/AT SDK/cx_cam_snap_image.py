######################################################################################
## @package : CxCamLib
#  @file : cx_cam_snap_image.py
#  @brief Python example of using AT Camera library.
#  @copyright (c) 2017, Automation Technology GmbH
#  @version 04.10.2017, AT: initial version
#
#  This example shows the steps necessary to snap a single image.
#  1. (Find and) connect a camera device.
#  2. Allocate and queue internal buffers.
#  3. Start acquisition.
#  4. Grab an image buffer.
#  5. Get image from buffer (and do some processing on the image data).
#  6. Queue back the image buffer.
#  7. Stop acquisition.
#  8. Cleanup.
#
######################################################################################

import numpy as np
from matplotlib import pyplot as plt
import cx.cx_cam as cam
import cx.cx_base as base

# 1. (Find and) connect a camera device.
# discover cameras, you can also filter for discovering only certain cameras, e.g. "filter://169.254.213.222"
result = cam.cx_dd_findDevices("", 2000, cam.CX_DD_USE_GEV | cam.CX_DD_USE_GEV_BROADCAST)
if result!=base.CX_STATUS_OK:
    print("cx_dd_findDevices returned error %d" % (result))
    exit(0)

# get device URI of first discovered device
result, uri = cam.cx_dd_getParam(0, "URI")

# connect the camera with given URI, you can also open the camera with a valid URI without the discovering step above
# e.g. uri = "gev://169.254.64.2?fd=true"

result, hDev = cam.cx_openDevice(uri)
if result!=base.CX_STATUS_OK:
    print("cx_openDevice(%s) returned error %d" % (uri, result))
    exit(-1)

# 2. Allocate and queue internal buffers.
result =  cam.cx_allocAndQueueBuffers(hDev, 16)
if result!=base.CX_STATUS_OK:
    print("cx_allocAndQueueBuffers(%d) returned error %d" % (hDev, result))

# 3. Start acquisition.
result =  cam.cx_startAcquisition(hDev)
if result!=base.CX_STATUS_OK:
    print("cx_startAcquisition(%d) returned error %d" % (hDev, result))

# Grab image buffer, wait for image with optional timeout
result, hBuffer = cam.cx_waitForBuffer(hDev, 1000)
if result != base.CX_STATUS_OK:
    print("cx_waitForBuffer returned error %d" % (result))

# get image from buffer
# the img object holds a reference to the data in the internal buffer, if you need the image data after cx_queueBuffer you need to copy it!
result, img = cam.cx_getBufferImage(hBuffer, 0)
if result != base.CX_STATUS_OK:
    print("cx_getBufferImage returned error %d" % (result))

# matplotlib image display, scaled with 3*std
img_mean = np.mean(img.data)
img_min = np.min(img.data)
img_max = np.max(img.data)
img_std = np.std(img.data)

plt.title("image")
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
result = cam.cx_freeBuffers(hDev)
if result!=base.CX_STATUS_OK:
    print("cx_freeBuffers returned error %d" % (result))

result = cam.cx_closeDevice(hDev)
if result!=base.CX_STATUS_OK:
    print("cx_closeDevice returned error %d" % (result))
