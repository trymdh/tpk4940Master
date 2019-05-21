import os
import re
import random
import numpy as np
import glob
from quaternion import*
np.set_printoptions(suppress=True)

def getUname():
    uname = os.getlogin()
    if uname == "trymdh":
        uname = uname + ".WIN-NTNU-NO"
    return uname
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
    
    H = np.dot(Kb,np.transpose(Ka))
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
    t = np.dot(np.linalg.inv(np.dot(C.T, C)), np.dot(C.T, d))
    
    return R,t.ravel()

def get_A_B(n):
    #A = transformation between camera poses i.e between T_CW_1 and T_CW_2
    #B = transform between EE pose, i.e between T_OE_1 and T_OE_2
    
    #n is number of pair combinations
    uname = getUname()
    T_CW_path = "C:/Users/" + str(uname) + "/OneDrive/tpk4940Master/Camera calibration May/Calibration parameters"
    T_OE_path = "C:/Users/" + str(uname) + "/OneDrive/tpk4940Master/Camera calibration May/EndEffectorPositions/T_pose"

    #T_OE is the tool frame as seen in robot base coordinates
    os.chdir(T_OE_path)
    T_OE_fnames = sortList(glob.glob(os.getcwd().replace("\\","/") + "/*.npy"))
    Ts_OE = []
    for fname in T_OE_fnames:
        T = np.load(fname)
        T[0:3,3] = np.around(T[0:3,3]*1000,decimals = 3) #convert from m to mm
        Ts_OE.append(T)
    Ts_OE = np.asarray(Ts_OE)
    print(Ts_OE)
    #T_CW is the calibration target origin in Camera frame coordinates
    os.chdir(T_CW_path)
    ts = np.loadtxt("TranslationVectors.txt") #The translation of the camera relative to the calibration target
    Rs = np.loadtxt("RotationMatrices.txt")
    Rs = Rs.reshape(int(Rs.shape[0]/3),3,3) #RotationMatrices.txt needs to be reshaped from (#,3) into (#/3,3,3) where "#" is the number of images
    Ts_CW = []
    for i in range(0,len(ts)):
        T_CW = np.eye(4)
        T_CW[0:3,0:3] = np.linalg.inv(Rs[i])
        T_CW[0:3,3] = ts[i]
        Ts_CW.append(T_CW)
    Ts_CW = np.asarray(Ts_CW)
    print(Ts_CW)
    #k = np.sort(np.random.choice(Ts_CW.shape[0],n,replace = False))
    #print(k)
    A = [] 
    B = []
    #calulate different pairs of poses
    for i in range(0,n):
        a = i
        b = i + 1
        print(a,b)
        A.append(np.dot(np.linalg.inv(Ts_OE[b]),Ts_OE[a]))
        B.append(np.dot(Ts_CW[b],np.linalg.inv(Ts_CW[a])))
    return A,B
    

n = 4
A,B = get_A_B(n)
Rx,tx = handEye(A,B)#Transform from end effector to camera frame
X = np.eye(4)
X[0:3,0:3],X[0:3,3] = np.around(Rx,decimals = 6), np.around(tx,decimals=3)
print(X)


#np.save("X.npy",X)
e_angle = []
e_vec = []
t_e = []
for i in range(0,len(A)):
    AX = np.dot(A[i],X)
    XB = np.dot(X,B[i])
    R_a,R_b = A[i][0:3,0:3], B[i][0:3,0:3]
    t_ax = AX[0:3,3]
    t_xb = XB[0:3,3]
    #calculate the deviation in rotation using quaternions
    q_a_est = shepperd(Rx@R_b@np.linalg.inv(Rx))
    q_a = shepperd(R_a)
    q_e = qprod(qconj(q_a_est),q_a)
    theta_e = 2*np.arccos(np.linalg.norm(q_e[0]))
    e_angle.append(theta_e)
    e_vec.append(q_e[1:])
    t_e.append(np.linalg.norm(t_xb - t_ax))

#print(rotx(pi/2)@roty(pi))
print("Mean error in rotation angle is {0} radians".format(np.mean(e_angle)))
print("Mean error in rotation axis [{1},{2},{3}]".format(np.mean(e_angle),np.mean(e_vec[0]),np.mean(e_vec[1]),np.mean(e_vec[2])))
print("Mean error in translation: {0} mm".format(np.mean(t_e)))
