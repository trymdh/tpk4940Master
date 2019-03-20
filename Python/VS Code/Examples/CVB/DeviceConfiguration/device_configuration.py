# CVBpy Example Script
#
# 1. Loads a GenICam device.
# 2. Get the device node map.
# 3. Modify the exposure time node
#
# Reaueiers: A connected and configured GenICam device


import os
import cvb

device = cvb.DeviceFactory.open(os.path.join(cvb.install_path(), "drivers", "GenICam.vin"), port=0)
dev_node_map = device.node_maps["Device"]

exposure_node = dev_node_map["ExposureTime"]
exposure_node.value = exposure_node.max / 2

print("Exposure time set to: " + str(exposure_node.value) + " " + exposure_node.unit)
