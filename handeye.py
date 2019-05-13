import os
import re
import random
import numpy as np
import glob
np.set_printoptions(suppress=True)

def sortList(unsortedList):
    #sort a list in alphanumeric order
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)',key)]
    return sorted(unsortedList,key = alphanum_key)

def skew(k):
    return np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])

def unskew(SS):
    """
    This function takes in a skew symmetrix matrix and returns
    it on vector form.
    """
    x = SS[2,1]
    y = SS[0,2]
    z = SS[1,0]
    return np.array([[x,y,z]]).T

def logMatrix(R):
    """
    A = |R(theta) t(x,y,z)|
        |   0        1    |
    when |theta| < pi:
        tr(R) = 1 + 2*cos(theta)
    ----------------------------------------
    log A = (R-R.T)*(theta/2*sin(theta))
    """
    theta = np.arccos((np.trace(R)-1)/2)
    #print(np.linalg.norm(theta) < np.pi)
    
    log_A_skewsym = (R-R.T)*theta/(2*np.sin(theta))
    log_A = unskew(log_A_skewsym)
    return log_A

def handEye(A,B):
    """
    Set of transforms between the robot base and the robot end effector:
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
    S = np.eye(3)
    S[2,2] = np.linalg.det(v@u.T)

    R = v@S@u.T
    
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

uname = os.getlogin()
if uname == "trymdh":
    uname = uname + ".WIN-NTNU-NO"
B_folder_path = "C:/Users/" + str(uname) + "/OneDrive/tpk4940Master/Camera calibration May/Calibration parameters"
A_folder_path = "C:/Users/" + str(uname) + "/OneDrive/tpk4940Master/Camera calibration May/EndEffectorPositions/T_pose"

#A = [A1 A2 ... An] :
os.chdir(A_folder_path)
A_fname = sortList(glob.glob(os.getcwd().replace("\\","/") + "/*.npy"))
Ts_OE = []
for fname in A_fname:
    Ts_OE.append(np.load(fname))
Ts_OE = np.asarray(Ts_OE)

#B = [B1 B2 ... Bn]: Transform between work space and camera frame at each pose 1,2,..,n
os.chdir(B_folder_path)
ts = np.loadtxt("TranslationVectors.txt")
Rs = np.loadtxt("RotationMatrices.txt")
Rs = Rs.reshape(int(Rs.shape[0]/3),3,3) #RotationMatrices.txt needs to be reshaped from (#,3) into (#/3,3,3) 
print(ts)
Ts_CW = []
for i in range(0,len(ts)):
    T = np.eye(4)
    T[0:3,0:3] = Rs[i]
    T[0:3,3] = ts[i]
    Ts_CW.append(np.linalg.inv(T))
Ts_CW = np.asarray(Ts_CW) # This matrix containts T_CW matrixes for 18 different poses

A = []
B = []
n = 100
for i in range(0,len(Ts_OE)):
    if len(A) == n:
        break
    for j in range(0,len(Ts_OE)):
        if j != i:
            if len(A) == n:
                break
            A_a = Ts_OE[i]
            A_b = np.linalg.inv(Ts_OE[j])
            B_a = np.linalg.inv(Ts_CW[i])
            B_b = Ts_CW[j]
            A.append(A_b@A_a)
            B.append(B_b@B_a)

X = handEye(A,B)
print(X)
R_e = []
t_e = []
for i in range(0,len(A)):
    R_e.append(np.linalg.norm((A[i]@X - X@B[i])[0:3,0:3]))
    t_e.append(np.linalg.norm((A[i]@X - X@B[i])[0:3,3]))
print("Mean error in rotation: {0}".format(np.mean(R_e)))
print("Mean error in translation: {0}".format(np.mean(t_e)))

  
from numpy import dot, eye, zeros, outer
from numpy.linalg import inv

def log(R):
    # Rotation matrix logarithm
    theta = np.arccos((R[0,0] + R[1,1] + R[2,2] - 1.0)/2.0)
    return np.array([R[2,1] - R[1,2], R[0,2] - R[2,0], R[1,0] - R[0,1]]) / (2*np.sin(theta))

def invsqrt(mat):
    u,s,v = np.linalg.svd(mat)
    return u.dot(np.diag(1.0/np.sqrt(s))).dot(v)

def calibrate(A, B):
    #transform pairs A_i, B_i
    N = len(A)
    M = np.zeros((3,3))
    for i in range(N):
        Ra, Rb = A[i][0:3, 0:3], B[i][0:3, 0:3]
        M += outer(log(Rb), log(Ra))

    Rx = dot(invsqrt(dot(M.T, M)), M.T)

    C = zeros((3*N, 3))
    d = zeros((3*N, 1))
    for i in range(N):
        Ra,ta = A[i][0:3, 0:3], A[i][0:3, 3]
        Rb,tb = B[i][0:3, 0:3], B[i][0:3, 3]
        C[3*i:3*i+3, :] = eye(3) - Ra
        d[3*i:3*i+3, 0] = ta - dot(Rx, tb)

    tx = dot(inv(dot(C.T, C)), dot(C.T, d))
    return Rx, tx.flatten()

R, t = calibrate(A,B)
X1 = np.eye(4)
X1[0:3,0:3] = R
X1[0:3,3] = t
print(X1)
R_e = []
t_e = []
for i in range(0,len(A)):
    R_e.append(np.linalg.norm((A[i]@X1 - X1@B[i])[0:3,0:3]))
    t_e.append(np.linalg.norm((A[i]@X1 - X1@B[i])[0:3,3]))
print("Mean error in rotation: {0}".format(np.mean(R_e)))
print("Mean error in translation: {0}".format(np.mean(t_e)))
