# CVBpy Example Script
#
# 1. Loads an image from file.
# 2. Resizes it using pixel mapping.
# 3 Save the file. 
#
# Reaueiers: -

import os
import cvb

src_image = cvb.Image(os.path.join(cvb.install_path(), "tutorial", "Clara.bmp"))

print("Loaded image size: " + str(src_image.width) + " x " + str(src_image.height))

dst_image = src_image.map(src_image.bounds, cvb.Size2D(src_image.size.width * 2, src_image.size.height  * 3))

print("Mapped image size: " + str(dst_image.width) + " x " + str(dst_image.height))

dst_image.save("Clara23.bmp")