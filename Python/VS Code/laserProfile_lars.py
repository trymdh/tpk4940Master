import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import cx.cx_cam as cam
import cx.cx_base as base
import time
from AT_cx_functions import*

hDev = getDevice()
cogConfig(hDev)

# get initial image
img_data = snap(hDev)
print(img_data.shape)
# for i in range(1000):
#     img_data = grabImage()
#     print(img_data / 64.0)
#     # time.sleep(0.1)
plt.gca().invert_yaxis()
plt.plot(img_data.ravel() / 64.0)
plt.show()
#save the laserlineprofile in an numpy-array
#np.save('laser.npy', img_data / 64.0)
a = 1
