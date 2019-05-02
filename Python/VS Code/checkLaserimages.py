from AT_cx_functions import*
import os
from matplotlib import pyplot as plt
wd = getCurrentWD()
laserimage_folder = wd + "/NewLaserImages"
print(wd)
os.chdir(laserimage_folder)

laserpixel = np.load("pixcoord_12.npy")

plt.gca().invert_yaxis()
plt.plot(laserpixel[:,1])
plt.show()
