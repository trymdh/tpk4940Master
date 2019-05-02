from AT_cx_functions import*
import numpy as np
import random

#Transform between world origin and the work space
"""
To simplify this can be defined in the base of the robot.
"""
T_OW = np.eye(4)
 
#Transform between world origin and robot end-effector
"""
If the joint vector q = [theta_1,theta_2,....,theta_n] is known, this transformation
can be calculated from IK
"""
T_OE = np.eye(4)

#Transform between end-effector and the camera frame.
"""
This is the desired output from the hand-eye calibration.
T_EC = X
"""
T_EC = np.eye(4)

#Transform between the work space and the camera frame
"""
This can be estimated after the camera has been calibrated,
using the camera matrix and tVecs, and rMats.
"""
T_WC = np.eye(4)
T_CW = np.linalg.inv(T_WC)

"""
The transformation of the benchmark point (work space origin), w.r.t the base
of the robot is:
T_OW = T_OE@T_EC@T_CW

---------------------------------------------------------------------
#One pair where q_1a != q_1b
Transform between the world origin and the robot end effector:
A_1a = T_OE_1a #Pose at joint vector q_1a
A_1b = T_OE_1b #Pose at joint vector q_1b

Transform between work space and camera frame at each pose:
B_1a = T_CW_1a #Pose at joint vector q_1a
B_1b = T_CW_1b #Pose at joint vector q_1b

The transformation between the robot base and the work space origin 
never changes, therefore:
T_OW_1a = T_OW_1b
or
A_1a@X@B_1a = A_1b@X@B_1b

Let 
A_1 = (A_1b)^(-1)@A_1a
B_1 = B_1b@(B_1a)^(-1)

Then the basic hand-eye equation for ONE pair of different poses is:
A_1@X = X@B_1
----------------------------------------------------------------------
To find X we need n >= 2 different pair of poses
A_1@X = X@B_1
A_2@X = X@B_2
    .....
A_n@X = X@B_n
"""

#Example with 2 poses
#Pair 1:
A_1 = np.array([
    [-0.989992, -0.141120, 0.000000, 0.000000],
    [0.141120, -0.989992, 0.000000,0.000000],
    [0.000000,0.000000,1.000000,0.000000],
    [0.000000,0.000000,0.000000,1.000000]
    ])
R_1A = A_1[0:3,0:3]; t_1A = A_1[0:3,3]

B_1 = np.array([
    [-0.989992, -0.138307, 0.028036, -26.9559],
    [0.138307, -0.911449, 0.38747,-96.1332],
    [-0.028036,0.387470,0.921456,19.4872],
    [0.000000,0.000000,0.000000,1.000000]
    ])
R_1B = B_1[0:3,0:3]; t_1B = B_1[0:3,3]

#Pair 2:
A_2 = np.array([
    [0.070732, 0.000000, 0.997495, -400.000],
    [0.000000, 1.000000, 0.000000, 0.000000],
    [-0.997495,0.000000,0.070737,400.000],
    [0.000000,0.000000,0.000000,1.000000]
    ])
R_2A = A_2[0:3,0:3]; t_2A = A_2[0:3,3]
B_2 = np.array([
    [0.070737, 0.198172, 0.997612, -309.543],
    [-0.198172, 0.963323, -0.180936, 59.0244],
    [-0.977612, -0.180936, 0.107415, 291.177],
    [0.000000,0.000000,0.000000,1.000000]
    ])
R_2B = B_2[0:3,0:3]; t_2B = B_2[0:3,3]

#Criteria for there to be a solution for AX = BX is that ||log A|| = ||log B||
"""
print(np.linalg.norm(logMatrix(A_1[0:3,0:3])))
print(np.linalg.norm(logMatrix(B_1[0:3,0:3])))
print(np.linalg.norm(logMatrix(A_2[0:3,0:3])))
print(np.linalg.norm(logMatrix(B_2[0:3,0:3])))
"""

alpha_1 = logMatrix(R_1A)
alpha_2 = logMatrix(R_2A)
beta_1 = logMatrix(R_1B)
beta_2 = logMatrix(R_2B)

AA = np.eye(3)
AA[0:3,0] = alpha_1.ravel()
AA[0:3,1] = alpha_2.ravel()
AA[0:3,2] = (skew(alpha_1)@alpha_2).ravel()

BB = np.eye(3)
BB[0:3,0] = beta_1.ravel()
BB[0:3,1] = beta_2.ravel()
BB[0:3,2] = (skew(beta_1)@beta_2).ravel()
BB_inv = np.linalg.inv(BB)

#Unknown roation matrix
R_x = AA@BB_inv
C = np.vstack(([R_1A - np.eye(len(R_1A))],[R_2A - np.eye(len(R_2A))])).reshape(6,3)
d = np.vstack([[R_x@t_1B - t_1A],[R_x@t_2B - t_2A]]).reshape(1,6)

#Uknown translation
t_x = np.linalg.inv(C.T@C)@C.T@d.T

#Solution
X = np.eye(4)
X[0:3,0:3] = R_x
X[0:3,3] = t_x.ravel()
print(X)

#hand-eye calibration:
def handEye(A,B):
    """
    Set of transforms between the work space and the robot end effector:
    A = [A1,A2,..,An]
    Set of transforms between between work space and camera frame:
    B = [B1,B2,...,Bn]

    One pair of A and B where q_1a != q_1b
    A1a = T_OE_1a #Pose at joint vector q_1a
    A1b = T_OE_1b #Pose at joint vector q_1b
    B1a = T_CW_1a #Pose at joint vector q_1a
    B1b = T_CW_1b #Pose at joint vector q_1b
    -> A1 = (A1b)^(-1)@A1a
    -> B1 = B1b@(B1a)^(-1)
    Then the basic hand-eye equation for ONE pair of different poses is:
    A1@X = X@B1
    To find X we need n >= 2 different pair of poses
    A1@X = X@B1
    A2@X = X@B2
        .....
    An@X = X@B_n
    """
    #Solving AX = BX, need n >= 2 pairs of transforms, i.e len(A) = len(B) >= 2
    #based on MATLAB code in Olav Vision Notes
    n = len(A)
    Ka = np.zeros((3,n)); Kb = np.zeros((3,n))
    for i in range(0,n):
        Ka[:,i] = logMatrix(A[i][0:3,0:3]).ravel()
        Kb[:,i] = logMatrix(B[i][0:3,0:3]).ravel()
        
    H = Kb@Ka.T
    u,s,vh = np.linalg.svd(H)
    v = vh.conj().T

    R = v@u.T
    C = []; d = []
    for i in range(0,n):
        C.append(A[i][0:3,0:3] - np.eye(3))
        d.append(R@B[i][0:3,3] - A[i][0:3,3])

    C = np.asarray(C).reshape(3*n,3)
    d = np.asarray(d).reshape(3*n,1)

    t1 = np.linalg.inv(C.T@C)
    t2 = C.T@d
    t = t1@t2

    X = np.eye(4)
    X[0:3,0:3] = R ; X[0:3,3] = t.ravel()
    
    return X

A = [A_1,A_2]
B = [B_1,B_2]
X_est = handEye(A,B)
print(X_est)
