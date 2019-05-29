import numpy as np

def error_checker(plane,point_cloud):
    [A,B,C,D] = plane
    distances = np.array([])
    for i in range(len(point_cloud[:,0])):
        [x,y,z] = [point_cloud[i,0],point_cloud[i,1],point_cloud[i,2]] 
        d = abs(A*x + B*y + C*z + D)/np.sqrt(A**2 + B**2 + C**2)
        distances = np.append(distances,d)
        
    return sum(distances)/len(distances)

def getError(pointCloud,pI,c):
    [A,B,C,D] = pI
    error_list = []
    for point in pointCloud:
        d = (A*(point[0]-c[0]) + B*(point[1]-c[1]) + C*(point[2]-c[2]))/np.sqrt(A**2 + B**2 + C**2)
        error_list.append(d)
    error_vec = np.array(error_list)
    median_error = np.median(error_vec)
    error_std = np.std(error_vec)
    #error_vec contains distances between each point in pointCloud and the plane pI
    return error_vec,median_error,error_std