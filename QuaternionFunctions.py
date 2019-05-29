import numpy as np
from MatrixFunctions import rotx,roty,rotz
def shepperd(R):
    #Quaternion from rotation matrix using Shepperd's algorithm,
    #which is stable, does not lose significant precision and uses only one sqrt.
    #J. Guidance and Control, 1 (1978) 223-224.
    q = np.transpose(np.zeros(4))
    z00 = R[0,0] + R[1,1] + R[2,2] # Trace of R
    z11 = R[0,0] + R[0,0] - z00
    z22 = R[1,1] + R[1,1] - z00
    z33 = R[2,2] + R[2,2] - z00
    #Find a large zii to avoid division by zero
    if z00 >= 0.5:
        w = np.sqrt(1.0 + z00)
        wInv = 1.0/w
        x = (R[2,1] - R[1,2])*wInv
        y = (R[0,2] - R[2,0])*wInv
        z = (R[1,0] - R[0,1])*wInv
    elif z11 >= 0.5:
        x = np.sqrt(1.0 + z11)
        xInv = 1.0/x
        w = (R[2,1] - R[1,2])*xInv
        y = (R[1,0] + R[0,1])*xInv
        z = (R[2,0] + R[0,2])*xInv
    elif z22 >= 0.5:
        y = np.sqrt(1.0 + z22)
        yInv = 1.0/y
        w = (R[0,2] - R[2,0])*yInv
        x = (R[1,0] + R[0,1])*yInv
        z = (R[2,1] + R[1,2])*yInv
    else:
        z = np.sqrt(1.0 + z33)
        zInv = 1.0/z
        w = (R[1,0] - R[0,1])*zInv
        x = (R[2,0] + R[0,2])*zInv
        y = (R[2,1] + R[1,2])*zInv
    eta = 0.5*w
    eps = 0.5*np.array([[x],[y],[z]])
    q[0] = eta
    q[1:] = np.transpose(eps)
    return q

def qprod(q1,q2):
    eta1 = q1[0] 
    eps1 = q1[1:] 
    eta2 = q2[0] 
    eps2 = q2[1:] 
    q = np.zeros(4)
    #formula (217) on page 47 in the notes.
    q[0] = eta1*eta2 - np.dot(eps1,eps2)
    q[1:] =  eta1*eps2 + eta2*eps1 + np.cross(eps1,eps2)
    return q

def qconj(q):
    q[1:] = -q[1:]
    return q
    
def quat2rot(q):
    #Takes a quaternion and returns its corresponding rotation matrix
    n = q[0]
    e_x,e_y,e_z = q[1],q[2],q[3]
    R = np.eye(3)
    R[0,0:3] = np.array([2*(n**2 + e_x**2)-1,2*(e_x*e_y - n*e_z),2*(e_x*e_z + n*e_y)])
    R[1,0:3] = np.array([2*(e_x*e_y + n*e_z),2*(n**2 + e_y**2)-1,2*(e_y*e_z - n*e_x)])
    R[2,0:3] = np.array([2*(e_x*e_z - n*e_y),2*(e_y*e_z + n*e_x),2*(n**2 + e_z**2)-1])
    return R