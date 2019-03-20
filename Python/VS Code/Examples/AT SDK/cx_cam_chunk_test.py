 import numpy as np
 import struct
 from matplotlib import pyplot as plt
 import cx.cx_cam as cam
 import cx.cx_base as base
 
 # discover cameras, you can also filter for discovering only certain cameras, e.g. "filter://169.254.213.222"
 result = cam.cx_dd_findDevices("", 2000, cam.CX_DD_USE_GEV | cam.CX_DD_USE_GEV_BROADCAST)
 if result!=base.CX_STATUS_OK:
     print("cx_dd_findDevices returned error %d" % (result))
     exit(0)
 
 # get device URI of first discovered device
 result, uri = cam.cx_dd_getParam(0, "URI")
 
 # connect the camera with given URI, you can also open the camera with a valid URI without the discovering step above
 # e.g. uri = "gev://169.254.239.221/?mac=00-50-C2-8E-DD-EE"
 result, hDev = cam.cx_openDevice(uri)
 if result!=base.CX_STATUS_OK:
     print("cx_openDevice(%s) returned error %d" % (uri, result))
     exit(-1)
 
 # configure camera
 aoi_w=1280   # für 1280=79clks, für >= 1040 = 62clks sonst 42clks (min pixel witdh 668, darunter wird hblank länger)
 aoi_h=200
 itime = 100
 line_rate = 4000.0    # 4kHz
 chunkmode = 1
 cam.cx_setParam(hDev, "PixelFormat", "Mono16")
 cam.cx_setParam(hDev, "Width", aoi_w)
 cam.cx_setParam(hDev, "OffsetX", 0) #(1280-aoi_w)/2)
 cam.cx_setParam(hDev, "CameraMode", "MaximumIntensity") # MaximumIntensity = 2
 cam.cx_setParam(hDev, "ProfilesPerFrame", 100)
 cam.cx_setParam(hDev, "AoiHeight", aoi_h)
 cam.cx_setParam(hDev, "AoiOffsetY", (1024-aoi_h)/2)
 cam.cx_setParam(hDev, "AoiThreshold", 0)    # standard 120
 cam.cx_setParam(hDev, "AcquisitionLineRateEnable", 1)   # When set to true, you can manually adjust the Acquisition Line Rate
 cam.cx_setParam(hDev, "AcquisitionLineRate", line_rate)
 cam.cx_setParam(hDev, "ExposureTime", itime)
 cam.cx_setParam(hDev, "ReverseX", 0)    # force update, after setting all parameters
 cam.cx_setParam(hDev, "ReverseY", 0)
 cam.cx_setParam(hDev, "ChunkModeActive", chunkmode)
 cam.cx_setParam(hDev, "ChunkModeSelector", "OneChunkPerProfile")    # OneChunkPerFrame, OneChunkPerProfile
 
 # maximize network throughput
 #cam.cx_setParam(hDev, "GevSCPSPacketSize", 8192)    # packetsize
 #cam.cx_setParam(hDev, "GevSCPD", 0) # interpacket delay
 
 # allocate and queue internal buffers
 result =  cam.cx_allocAndQueueBuffers(hDev, 16)
 if result!=base.CX_STATUS_OK:
     print("cx_allocAndQueueBuffers(%d) returned error %d" % (hDev, result))
 
 # start image acquisition
 result =  cam.cx_startAcquisition(hDev)
 if result!=base.CX_STATUS_OK:
     print("cx_startAcquisition(%d) returned error %d" % (hDev, result))
 
 # Grab image buffer, wait for image with optional timeout
 result, hBuffer = cam.cx_waitForBuffer(hDev, 1000)
 if result!=base.CX_STATUS_OK:
     print("cx_waitForBuffer returned error %d" % (result))
 
 # get image from buffer
 # the img object holds a reference to the data in the internal buffer, if you need the image data after cx_queueBuffer you need to copy it!
 result, img = cam.cx_getBufferImage(hBuffer, 0)
 if result!=base.CX_STATUS_OK:
     print("cx_getBufferImage returned error %d" % (result))
 
 frameInfo = base.CX_CHUNK_FRAME_INFO()
 ci = base.CX_CHUNK_CAMERA_INFO()
 
 if chunkmode == 1:
     
 
     # check if node is available (i.e. XML complies GenICam SFNC 2.3)
     result, profileIdx = cam.cx_getParam(hDev, "ChunkScanLineSelector")
 
     if result == base.CX_STATUS_OK:
         # get the number of profiles
         result, type = cam.cx_getParamInfo(hDev, cam.CX_PARAM_INFO_TYPE, "ChunkScanLineSelector")
         if result != base.CX_STATUS_OK:
             exit()
         result, index_range = cam.cx_getParamInfo(hDev, cam.CX_PARAM_INFO_RANGE, "ChunkScanLineSelector")
         if result != base.CX_STATUS_OK:
             exit()
 
         numProfiles = index_range[1] - index_range[0] + 1
 
         # read chunk data for each profile
         timestamp = np.zeros((numProfiles), dtype=np.uint64)
         framecount = np.zeros((numProfiles), dtype=np.uint32)
         rel_framecount = np.zeros((numProfiles), dtype=np.uint32)
         aoi_ys = np.zeros((numProfiles), dtype=np.uint32)
 
         for profileIdx in range(index_range[0],index_range[1] + 1):
             result = cam.cx_setParam(hDev, "ChunkScanLineSelector", profileIdx)
             if result != base.CX_STATUS_OK:
                 exit()
 
             result, timestamp[profileIdx - index_range[0]] = cam.cx_getParam(hDev, "ChunkTimestamp")
             result, framecount[profileIdx - index_range[0]] = cam.cx_getParam(hDev, "ChunkFramecounter")
             result, aoi_ys[profileIdx - index_range[0]] = cam.cx_getParam(hDev, "ChunkEncoderValue")
 
         plt.figure(3)
         plt.plot(framecount)
 
     
 
     # get number of chunks
     result, num_chunk = cam.cx_getBufferInfo(hBuffer, cam.CX_BUFFER_INFO_NUM_CHUNK)
     if result != base.CX_STATUS_OK:
         exit()
 
     # iterate chunk data
     for chunk_no in range(0,num_chunk):
         result, chunk = cam.cx_getBufferChunk(hBuffer, chunk_no)
         if result != base.CX_STATUS_OK:
             exit()
 
         print("chunk_no:%d, chunk.descriptor:%x, chunk.length:%d" % (chunk_no, chunk.descriptor, chunk.data.size))
 
         # extract chunk frame info
         if chunk.descriptor == base.CX_CHUNK_FRAME_INFO_ID:
             print("Chunk Frame Info descriptor:%X, length:%d" % (chunk.descriptor, chunk.data.size))
             print("sizeYReal:%d, numAcqInfo:%d, flag:%X" % (
             frameInfo.sizeYReal, frameInfo.numChunkAcqInfo, frameInfo.flag))
             frameInfo.sizeYReal, frameInfo.numChunkAcqInfo, frameInfo.flag = struct.unpack_from('<III', chunk.data,
                                                                                                 0)  # little-endian
             print("sizeYReal:%d, numAcqInfo:%d, flag:%X" % (frameInfo.sizeYReal, frameInfo.numChunkAcqInfo, frameInfo.flag))
 
         # extract chunk camera info
         if chunk.descriptor == base.CX_CHUNK_CAMERA_INFO_ID:
             N = frameInfo.numChunkAcqInfo;
             print("Chunk Camera Info descriptor:%X, length:%d" % (chunk.descriptor, chunk.data.size))
             timestamp = np.zeros((N), dtype=np.uint64)
             framecount = np.zeros((N), dtype=np.uint32)
             aoi_ys = np.zeros((N), dtype=np.uint32)
 
             for i in range(N):
                 ci.timeStamp64L, ci.timeStamp64H, ci.frameCnt, ci.triggerCoord, ci.triggerStatus, ci.AO0, ci.AI0, ci.INT_idx, ci.AOI_idx, ci.AOI_ys, ci.AOI_dy, ci.AOI_xs, ci.AOI_trsh, ci.AOI_alg = struct.unpack_from(
                     '<IIIiBHHBBHHHHB', chunk.data, i * 32)  # little-endian
                 #print(ci.timeStamp64L,ci.timeStamp64H,ci.frameCnt,ci.triggerCoord,ci.triggerStatus,ci.AO0,ci.AI0,ci.INT_idx,ci.AOI_idx,ci.AOI_ys,ci.AOI_dy,ci.AOI_xs,ci.AOI_trsh,ci.AOI_alg)
                 timestamp[i] = ci.timeStamp64L + ci.timeStamp64H * (2 ** 32)
                 framecount[i] = ci.frameCnt
                 aoi_ys[i] = ci.AOI_ys
 
             plt.figure(2)
             plt.plot(framecount)
 
 plt.figure(1)
 plt.imshow(img.data, cmap='gray', interpolation='bicubic')
 plt.show()
 
 # stop acquistion
 result = cam.cx_stopAcquisition(hDev)
 if result!=base.CX_STATUS_OK:
     print("cx_stopAcquisition returned error %d" % (result))
 
 # cleanup
 result = cam.cx_freeBuffers(hDev)
 if result!=base.CX_STATUS_OK:
     print("cx_freeBuffers returned error %d" % (result))
 
 result = cam.cx_closeDevice(hDev)
 if result!=base.CX_STATUS_OK:
     print("cx_closeDevice returned error %d" % (result))