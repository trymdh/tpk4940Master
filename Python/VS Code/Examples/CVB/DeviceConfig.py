import os
import cvb 

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
#isNodeWritable(device_node_map["AoiTrackingEnable"])

#COG node
#print(camera_mode.entries[1])

resetNode = device_node_map["DeviceReset"]
a = 2
