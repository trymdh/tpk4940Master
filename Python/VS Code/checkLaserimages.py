from AT_cx_functions import*
import os
from matplotlib import pyplot as plt
wd = getCurrentWD()
laserimage_folder = wd + "/Newlaserimage"
os.chdir(laserimage_folder)

laserpixel = np.load("pixcoord_3.npy")
plt.gca().invert_yaxis()
plt.plot(laserpixel)
plt.show()
