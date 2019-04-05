from AT_cx_functions import*
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cv2 as cv2
import msvcrt

def triang(pix):
    t = time.time()
    ret,K,tvecs,rMats,dist= loadCaliParam()
    laser = np.load('C:/Users/TrymAsus/OneDrive/tpk4940Master/Python/VS Code/RansacPlane.npy')
    
    #1 Undistort the pixels
    pix = pix.reshape(-2, 1, 2)
    undistorted_pixels = cv2.undistortPoints(pix, K, dist).reshape(-1,2)

    #2 Triangulate
    ext_points = np.array([])

    #Define lasernormal and laserpoint
    ln = laser[0:3]
    lp = laser[3:6]

    l_0 = np.array([0,0,0])
    
    for coord in undistorted_pixels:
        coord = np.append(coord,1)
        l = coord/np.linalg.norm(coord)
        num = np.dot((lp - l_0), ln)
        deno = np.dot(l,ln)
        d = num/deno
        fullcoord = np.array([l * d]) + l_0
        ext_points = np.append(ext_points,fullcoord) 
    elapsed = time.time() - t

    return np.reshape(ext_points,(-1,3))

def in1d_dot_approach(A,B):
    cumdims = (np.maximum(A.max(),B.max())+1)**np.arange(B.shape[1])
    return A[~np.in1d(A.dot(cumdims),B.dot(cumdims))]

#Find and configure the device
hDev = getDevice()
cogConfig(hDev)

dead_array = np.zeros((2048,2))

for i in range(0,2048):
    dead_array[i,0] = i
#dead array = [[0,0],[1,0],[2,0],...,[2048,0]]
dead_points = triang(dead_array)

for i in range(0,1000):
    if msvcrt.kbhit():
        key = str(msvcrt.getch()).replace("b'","").replace("'","")
        if key == 'q':
            print("Loop exited")
            break

    #Snap an image of the scene in COG mode.
    laserlinepixels = snap(hDev)
    #reshape the COG-image from (2048,) to (2048,2)
    laserpixel = pixCoordify(laserlinepixels.ravel() / 64 , 2048)
    points = triang(laserpixel)

    #Removes non registered pixels from the triangulated points
    clean_points = in1d_dot_approach(points,dead_points)

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
    #plt.gca().invert_zaxis()
    ax.set_xlim3d(-200, 200)
    ax.set_ylim3d(-100,100)
    ax.set_zlim3d(0,800)

    ax.scatter3D(X,Y,Z)
    plt.pause(1/60)
    