import numpy as np
import cvb
import os
import sys
import matplotlib.pyplot as plt
import cv2
import glob

print(sys.version)
print(cvb.version())

#number of images we want to acquire
num_img = 1

with cvb.DeviceFactory.open(os.environ["CVB"] + "/drivers/GenICam.vin") as device:
    stream = device.stream
    stream.start()
    try:
        for i in range(0,num_img):
            image, status = stream.wait_for(1000)
            if status == cvb.WaitStatus.Ok:
                print("Acquired image " + str(i) + " into buffer " + 
                   str(image.buffer_index) + ".")
                img_str = 'snap'+str(i)+".bmp"
                image.save(img_str)
            else:
                raise RuntimeError("timeout during wait" 
                    if status == cvb.WaitStatus.Timeout else 
                        "acquisition aborted")
    except:
        pass 
    finally:
        stream.stop()

image = glob.glob("C:/Users/trymdh.WIN-NTNU-NO/*.bmp")
img = cv2.imread(image[0])
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

#filter the image, all pixels with gray intensity lower than the threshold is set to 0 (black)
threshold = 225

for u in range(0,len(gray)-1):
    for v in range(0,len(gray[u]-1)):
        if gray[u][v] <= threshold:
            gray[u][v] = 0

#resize image by half
gray = cv2.resize(gray,(0,0),fx = 0.5, fy=0.5)

#save the filtered and resized image
img_str = "filteredAndResized_snap.jpg"
cv2.imwrite(img_str,gray)

#pixel_coord is a list containing the pixel coordinates of the pixels with 
#gray intensity higher than the threshold

radius = 1
pixel_coord = []
for u in range(0,len(gray)-1):
    for v in range(0,len(gray[u]-1)):
        if gray[u][v] >= threshold:
            pixel_coord.append([u,v])

#to illustrate that we have the coordinates of the points along the laser line
#circle are drawn
for i in pixel_coord:
    u = i[1]
    v = i[0]
    cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    line = cv2.circle(gray,(u,v),radius,(255,0,0))


cv2.imshow('line',line)
cv2.waitKey(0)
    
#Line detection using opencv and probabilistic Hough Line Transform 
edges = cv2.Canny(gray,50,200)
cedges = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
cedgesP = np.copy(cedges)
lines = cv2.HoughLines(edges,1,np.pi/180,50,None,50,10)

if lines is not None:
    for i in range(0, len(lines)):
        rho = lines[i][0][0]
        theta = lines[i][0][1]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
        pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
        
        cv2.line(cedges, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
        
linesP = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, None, 50, 10)

if linesP is not None:
    for i in range(0, len(linesP)):
        l = linesP[i][0]
        cv2.line(cedgesP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
    
cv2.imshow("Source", gray)
cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cedgesP)
    
cv2.waitKey()
