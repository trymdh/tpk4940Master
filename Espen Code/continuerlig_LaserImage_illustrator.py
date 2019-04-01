from AT_cx_functions import*
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from TriangulateReading import triang
import cv2 as cv2

#Find and configure the device
hDev = getDevice()
cogConfig(hDev)

dead_array = np.zeros((2048,2))
for i in range(0,2048):
    dead_array[i,0] = i

dead_points = triang(dead_array)

def in1d_dot_approach(A,B):
    cumdims = (np.maximum(A.max(),B.max())+1)**np.arange(B.shape[1])
    return A[~np.in1d(A.dot(cumdims),B.dot(cumdims))]

for i in range(0,1000):

    #Snap an image of the scene in COG mode.
    laserlinepixels = snap(hDev)
    #reshape the COG-image from (2048,) to (2048,2)
    laserpixel = pixCoordify(laserlinepixels.ravel() / 64 , 2048)
    
    points = triang(laserpixel)

    clean_points = in1d_dot_approach(points,dead_points) #Removes non registered pixels from the triangulated points

    plt.clf()
    plt.ion()
    fig = plt.figure(1)
    ax = plt.axes(projection='3d')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    X = clean_points[:,0]
    Y = clean_points[:,1]
    Z = clean_points[:,2]
    ax.set_xlim3d(-200, 200)
    ax.set_ylim3d(-100,100)
    ax.set_zlim3d(0,800)

    ax.scatter3D(X,Y,Z)
    plt.pause(1/60)
    








