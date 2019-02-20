import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import cx.cx_cam as cam
import cx.cx_base as base

#find connected devices 
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

#Allocate and queue internal buffers (buffer = temporary image storage)
result = cam.cx_allocAndQueueBuffers(hDev,16)

#Start acquisition
result = cam.cx_startAcquisition(hDev)
if result != base.CX_STATUS_OK:
    print("Error, acquisition not started due to error")

fig = plt.figure()
plt.title("sequence")

#Snap an image sequence of 100 images from the device    
images = []
num_img = 100

for x in range(0,num_img-1):
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

"""
Til nå har er kameraet funnet og det er mulig å ta bilder. Det som må gjøres videre er å finne en måte å manipulere kameraparameterene på, slik at det er mulig
å sette forskjellig image mode.
"""

def isNodeWritable(node):
    if node.is_writeable:
        print(str(node.name) + " is writeable")
    else:
        print(str(node.name) + " is not writeable")

#path to current work directory
current_work_directory = os.getcwd().replace("\\","/")

device = cvb.DeviceFactory.open(os.environ["CVB"] + "/drivers/GenICam.vin")
device_node_map = device.node_maps["Device"]

camera_mode = device_node_map["CameraMode"]
print(camera_mode.value)

#note that AOI tracking is only writable in 3D-mode (COG,THRS,MAX), and not in image mode.
isNodeWritable(device_node_map["AoiTrackingEnable"])

print(camera_mode.entries[1])
a = 2
"""
Har funnet ut at kamera instillingene ligger i såkalte nodes, og kan bli manipulert via device-node_maps i python. Har ikke funnet ut hvilken property av camera_mode noden
som skal endres for å få kameraet over i COG mode. Default-instillingen er image mode. Bruk debuggeren i VS code til å se node propertisene..
"""
