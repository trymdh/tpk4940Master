######################################################################################
## @package : CxCamLib
#  @file : cx_cam_snap_sequence.py
#  @brief Python example of using AT Camera library.
#  @copyright (c) 2017, Automation Technology GmbH.
#  @version 04.10.2017, AT: initial version
#
#  This example shows the steps necessary to snap a sequence of images.
#  1. (Find and) connect a camera device.
#  2. Allocate and queue internal buffers.
#  3. Start acquisition.
#  4. LOOP
#  		1. Grab an image buffer.
#  		2. Get image from buffer (and do some processing on the image data, here copy the data).
#  		3. Queue back the image buffer.
#  5. Stop acquisition.
#  6. Cleanup.
######################################################################################

import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import cx.cx_cam as cam
import cx.cx_base as base

# 1. (Find and) connect a camera device.
# discover cameras, you can also filter for discovering only certain cameras, e.g. "filter://169.254.213.222"
result = cam.cx_dd_findDevices("", 2000, cam.CX_DD_USE_GEV | cam.CX_DD_USE_GEV_BROADCAST)
if result != base.CX_STATUS_OK:
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

# 2. Allocate and queue internal buffers.
result =  cam.cx_allocAndQueueBuffers(hDev, 16)
if result!=base.CX_STATUS_OK:
    print("cx_allocAndQueueBuffers(%d) returned error %d" % (hDev, result))

# 3. Start acquisition.
result =  cam.cx_startAcquisition(hDev)
if result!=base.CX_STATUS_OK:
    print("cx_startAcquisition(%d) returned error %d" % (hDev, result))

fig = plt.figure()
plt.title("sequence")
images = []

# 4. LOOP: snap image sequence with 100 images
for x in range(0,99):
    # 4.1 Grab image buffer, wait for image with optional timeout
    result, hBuffer = cam.cx_waitForBuffer(hDev, 1000)
    if result != base.CX_STATUS_OK:
        print("cx_waitForBuffer returned error %d" % (result))

    # 4.2 get image from buffer
	# the img object holds a reference to the data in the internal buffer, if you need the image data after cx_queueBuffer you need to copy it!
    result, img = cam.cx_getBufferImage(hBuffer, 0)
    if result != base.CX_STATUS_OK:
        print("cx_getBufferImage returned error %d" % (result))

    # matplotlib image display, scaled with 3*std
    img_mean = np.mean(img.data)
    img_min = np.min(img.data)
    img_max = np.max(img.data)
    img_std = np.std(img.data)

    # render the image
    axes_img = plt.imshow(img.data, vmin=img_mean - 3 * img_std, vmax=img_mean + 3 * img_std, cmap='gray',
                          interpolation='None', animated=True)
    # add a colorbar
    if x == 0:
        plt.colorbar()

    # append image data to array
    images.append([axes_img])

    # 4.3 Queue back the buffer. From now on the img.data is not valid anymore and might be overwritten with new image data at any time!
    result = cam.cx_queueBuffer(hBuffer)
    if result != base.CX_STATUS_OK:
        print("cx_queueBuffer returned error %d" % (result))

# Display the image sequence
ani = animation.ArtistAnimation(fig, images, interval=20, blit=True, repeat_delay=1000)
#ani.save('dynamic_images.mp4')
plt.show()

# 5. Stop acquisition.
result = cam.cx_stopAcquisition(hDev)
if result!=base.CX_STATUS_OK:
    print("cx_stopAcquisition returned error %d" % (result))

# 6. cleanup
result = cam.cx_freeBuffers(hDev)
if result!=base.CX_STATUS_OK:
    print("cx_freeBuffers returned error %d" % (result))

result = cam.cx_closeDevice(hDev)
if result!=base.CX_STATUS_OK:
    print("cx_closeDevice returned error %d" % (result))
