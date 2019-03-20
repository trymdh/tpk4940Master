# CVBpy Example Script
#
# 1. Loads an image from file.
# 2. Converts it to a numpy array without copying
# 3. Modify pixel values through numpy
# 4. Save the image. 
#
# Reaueiers: numpy

import os
import cvb

image = cvb.Image(os.path.join(cvb.install_path(), "tutorial", "Clara.bmp"))

# copy=Fasle is default, but just a request
np_array = cvb.as_array(image, copy=False)

print("Data was copyied: " + str(np_array.flags["OWNDATA"]))

# pixel access
np_array[83 : 108, 48 : 157] = 0

image.save("ClaraUnknown.bmp")

