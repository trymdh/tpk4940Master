import numpy as np
import cvb
import os
import sys
import matplotlib.pyplot as plt
import cv2
import glob

#print(sys.version)
#print(cvb.version())

#path to current work directory
current_work_directory = os.getcwd().replace("\\","/")

images = glob.glob(current_work_directory + "/*.bmp")

#acquire images from C4 camera if there are no .bmp images
#in the work directory
if not images:
    with cvb.DeviceFactory.open(os.environ["CVB"] + "/drivers/GenICam.vin") as device:
        stream = device.stream
        stream.start()
        num_img = 1
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

def thresholdFilter(image_list,threshold):
    for image in image_list:
        image = cv2.imread(image)
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        height, width = gray.shape[:2]
        for u in range(0,height-1):
            for v in range(0,width-1):
                if gray[u][v] <= threshold:
                    gray[u][v] = 0
                elif gray[u][v] >= threshold:
                    gray[u][v] = 255
        cv2.imwrite("threshold_" + str(threshold) + "_filteredgrey.jpg",gray)
    return gray
        
def getPixCoord(image,threshold):
    pixel_coord = []
    height, width = image.shape[:2]
    if len(pixel_coord) is not 0:
        pixel_coord = []
    else:
        for u in range(0,height-1):
            for v in range(0,width-1):
                if gray[u][v] >= threshold:
                    pixel_coord.append([u,v])
    return pixel_coord

def drawCircle(image,radius,pixel_coordinates):
    for coordinates in pixel_coordinates:
        u = coordinates[1]
        v = coordinates[0]
        line = cv2.circle(image,(u,v),radius,(255,0,0))
    return line

def probHoughLine(image):

    #edge detection using Canny Edge
    edges = cv2.Canny(image,50,200)
    
    #converting from grayscale to RGB image
    cEdges = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    cEdgesP = np.copy(cEdges)
    
    #line detection
    lines = cv2.HoughLines(edges,1,np.pi/180,50,None,50,10)
    if lines is not None:
        for i in range(0,len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv2.line(cEdges,pt1,pt2,(0,0,255), 3, cv2.LINE_AA)
    
    linesP = cv2.HoughLinesP(edges,1,np.pi/180,50,None,50,10)
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv2.line(cEdgesP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

    return cEdgesP
    
#filtering
image_list = glob.glob(current_work_directory + "/*.bmp")

threshold = 200
gray = thresholdFilter(image_list,threshold)

#localization of points along line
pix_coord = getPixCoord(gray,threshold)

#to check if the found points is on the line 
#circle are drawn with center in each point.
line = drawCircle(gray,5,pix_coord)
cv2.imwrite("foundline.jpg",line)

#probabilistic Hough Line transfrom
gray = thresholdFilter(image_list,threshold)
cEdgesP = probHoughLine(gray)
cv2.imwrite("houghLineDetection.jpg",cEdgesP)

