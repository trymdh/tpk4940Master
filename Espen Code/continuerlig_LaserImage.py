from AT_cx_functions import*
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from TriangulateReading import triang
import cv2 as cv2

#Find and configure the device
hDev = getDevice()
cogConfig(hDev)



for i in range(0,1000):

    #Snap an image of the scene in COG mode.
    laserlinepixels = snap(hDev)
    #reshape the COG-image from (2048,) to (2048,2)

    laserpixel = pixCoordify(laserlinepixels.ravel() / 64 , 2048)
    
    points = triang(laserpixel)
    plt.clf()
    plt.ion()
    fig = plt.figure(1)
    ax = plt.axes(projection='3d')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    X = points[:,0]
    Y = points[:,1]
    Z = points[:,2]
    ax.set_zlim(0,max(Z))
    ax.scatter3D(X,Y,Z)
    plt.pause(1/60)
    








