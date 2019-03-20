# CVBpy Example Script
#
# 1. Open the GenICam.vin driver.
# 2. Acquire images in a dedicated native thread through a stream handler.
# 3. Measure the frame rate and prit results.
#
# Reaueiers: A connected and configured GenICam device

import time
import os
import cvb

class MyStreamHandler(cvb.SingleStreamHandler):

    def __init__(self, stream):
        super().__init__(stream)
        self.rate_counter = cvb.RateCounter()

    # called in the interpreter thread to setup additionla stuff.
    def setup(self, stream):
        super().setup(stream)
        print("setup")

    # called in the interpreter thread to tear down stuff from setup.
    def tear_down(self, stream):
        super().tear_down(stream)
        print("tear_down")

    # called from the aqusition thread
    def handle_async_stream(self, stream):
        super().handle_async_stream(stream)
        print("handle_async_stream")

    # called from the aqusition thread
    def handle_async_wait_result(self, image, status):
        super().handle_async_wait_result(image, status)
        self.rate_counter.step()
        print("New image: " + image.__class__.__name__ + " " + str(image) + " | Status: " + str(status) + " | Buffer Index: " + str(image.buffer_index))

    # print messurement results
    def eval(self):
        print("Acquired with: " + str(self.rate_counter.rate) + " fps")



device = cvb.DeviceFactory.open(os.environ["CVB"] + "/drivers/GenICam.vin")

handler = MyStreamHandler(device.stream)

handler.run()

time.sleep(10)

handler.finish()


handler.eval()
