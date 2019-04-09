from AT_cx_functions import*
from matplotlib import pyplot as plt

#Find and configure the device
hDev = getDevice()
cogConfig(hDev)

plt.gca().invert_yaxis()
for i in range(0,1000):

    #Snap an image of the scene in COG mode.
    laserlinepixels = snap(hDev)
    #reshape the COG-image from (2048,) to (2048,2)
    laserpixel = pixCoordify(laserlinepixels.ravel() / 64 , 2048)

    plt.plot(laserpixel)
    plt.pause(1/(3*60))
plt.show()
