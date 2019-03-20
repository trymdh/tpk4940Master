# CVBpy Example Script
#
# 1. Discover all GenICam devices.
# 2. Load the first device found.
# 3. Acquire images.
#
# Reaueiers: A connected GenICam device


import cvb


discover = cvb.DeviceFactory.discover_from_root(cvb.DiscoverFlags.IgnoreVins)

with cvb.DeviceFactory.open(discover[0].access_token) as device:
    
    stream = device.stream
    stream.start()

    for i in range(10):
        image, status = stream.wait()
        if status == cvb.WaitStatus.Ok:
            print("Acquired image: " + str(i))

    stream.abort()

