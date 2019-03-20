# CVBpy Example Script
#
# 1. Open the GenICam.vin driver.
# 2. Acquire images.
#
# Reaueiers: A connected and configured GenICam device

import os
import cvb
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import cv2
import time
import numpy as np

with cvb.DeviceFactory.open(os.path.join(cvb.install_path(), "drivers", "GenICam.vin"), port=0) as device:
    
    stream = device.stream
    stream.start()
    images = []
    fig = plt.figure()
    plt.title("Camera stream")

    for i in range(100):
        img, status = stream.wait(1000)
        if status == cvb.WaitStatus.Ok:
            img_mean = np.mean(img)
            img_min = np.min(img)
            img_max = np.max(img)
            img_std = np.std(img)
            axes_img = plt.imshow(img, vmin = img_mean - 3*img_std, vmax = img_mean + 3*img_std, cmap = "gray", interpolation="None", animated = True)
            images.append([axes_img])
    
    stream.abort()
    
    #show the sequence
    ani = animation.ArtistAnimation(fig,images,interval = 20, blit = True, repeat_delay = 1000)
    plt.show()
    

