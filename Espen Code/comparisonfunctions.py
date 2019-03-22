import numpy as np

#Assumes plane on the form Ax + By + Cz + D = 0 formatted as [A,B,C,D], checks distance normal
#Formulas collected from https://mathinsight.org/distance_point_plane
def error_checker(plane,point_cloud):
    [A,B,C,D] = plane
    distances = np.array([])
    for i in range(len(point_cloud[:,0])):
        [x,y,z] = [point_cloud[i,0],point_cloud[i,1],point_cloud[i,2]] 

        d = abs(A*x + B*y + C*z + D)/np.sqrt(A**2 + B**2 + C**2)
        distances = np.append(distances,d)
    return sum(distances)/len(distances)

#Checks the difference in plane-value and cloud z-value
def fdiff_checker(plane,point_cloud):
    [A,B,C,D] = plane
    errors = np.array([])
    for i in range(len(point_cloud[:,0])):
        error = point_cloud[i,2] - (A*point_cloud[i,0] + B*point_cloud[i,1] + D)/C
        errors = np.append(errors,error)
    return sum(errors)/len(errors)
