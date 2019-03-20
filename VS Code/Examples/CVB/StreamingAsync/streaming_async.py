# CVBpy Example Script
#
# 1. Open the GenICam.vin driver twice on port 1 and port 0.
# 2. Asyncronously acquire images.
# 3. Measure the frame rate and prit results.
#
# Reaueiers: Two connected GenICam devices, which support pending images.


import os
import asyncio
import cvb




rate_counters = [None, None]

async def async_acquire(port):
    with cvb.DeviceFactory.open((os.environ["CVB"] + "/drivers/GenICam.vin"), port=port) as device:
        stream = device.stream
        stream.start()

        rate_counter = cvb.RateCounter()

        for i in range(0, 100):
            result = await  stream.wait_async()
            image, status = result.value
            rate_counter.step()
            if status == cvb.WaitStatus.Ok:
                print("Buffer index: " + str(image.buffer_index) + " from stream: " + str(port))

        rate_counters[port] = rate_counter
 

        stream.abort()
    



watch = cvb.StopWatch()

loop = asyncio.get_event_loop()
loop.run_until_complete(asyncio.gather(
    async_acquire(port=1),  
    async_acquire(port=0))) 
loop.close()

duration = watch.time_span

print("Acquired on port 0 with " + str(rate_counters[0].rate) + " fps")
print("Acquired on port 1 with " + str(rate_counters[1].rate) + " fps")
print("Overall measurement time: " +str(duration / 1000) + " seconds")


