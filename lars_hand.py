import numpy
from numpy import dot, eye, zeros, outer
from numpy.linalg import inv
import glob
import re
import os

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

def get_A_B(n):
    #n is number of pair combinations
    uname = getUname()
    T_CW_path = "C:/Users/" + str(uname) + "/OneDrive/tpk4940Master/Camera calibration May/Calibration parameters"
    T_OE_path = "C:/Users/" + str(uname) + "/OneDrive/tpk4940Master/Camera calibration May/EndEffectorPositions/T_pose"

    #T_OE is the end effector frame as seen in robot base coordinates
    os.chdir(T_OE_path)
    T_OE_fnames = sortList(glob.glob(os.getcwd().replace("\\","/") + "/*.npy"))
    Ts_OE = []
    for fname in T_OE_fnames:
        T = numpy.load(fname)
        T[0:3,3] = T[0:3,3]*1000
        Ts_OE.append(T)
    Ts_OE = numpy.asarray(Ts_OE)
    print(Ts_OE[9])

    #T_CW is the calibration target origin in Camera frame coordinates
    os.chdir(T_CW_path)
    ts = numpy.loadtxt("TranslationVectors.txt") #The translation of the camera relative to the calibration target
    Rs = numpy.loadtxt("RotationMatrices.txt")
    Rs = Rs.reshape(int(Rs.shape[0]/3),3,3) #RotationMatrices.txt needs to be reshaped from (#,3) into (#/3,3,3) where "#" is the number of images
    Ts_CW = []
    for i in range(0,len(ts)):
        T_CW = eye(4)
        T_CW[0:3,0:3] = Rs[i]
        T_CW[0:3,3] = ts[i]
        Ts_CW.append(T_CW)
    Ts_CW = numpy.asarray(Ts_CW)
    A = []
    B = []
    #calulate different pairs of poses i and j.
    for i in range(0,len(Ts_OE)):
        if len(A) == n:
            break
        for j in range(0,len(Ts_OE)):
            if j != i:
                if len(A) == n:
                    break
                A.append(Ts_OE[j]@inv(Ts_OE[i]))
                B.append(inv(Ts_CW[j])@Ts_CW[i])
    return A,B

def log(R):
    # Rotation matrix logarithm
    theta = numpy.arccos((R[0,0] + R[1,1] + R[2,2] - 1.0)/2.0)
    return numpy.array([R[2,1] - R[1,2], R[0,2] - R[2,0], R[1,0] - R[0,1]]) / (2*numpy.sin(theta))

def invsqrt(mat):
    u,s,v = numpy.linalg.svd(mat)
    return u.dot(numpy.diag(1.0/numpy.sqrt(s))).dot(v)

def calibrate(A, B):
    #transform pairs A_i, B_i
    N = len(A)
    M = numpy.zeros((3,3))
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

n = 10
A,B = get_A_B(n)
Rx,tx = calibrate(A,B)
X = numpy.eye(4)
X[0:3,0:3] = Rx
X[0:3,3] = tx
print(X)

R_e = []
t_e = []
for i in range(0,len(A)):
    R_a = (A[i]@X)[0:3,0:3]
    R_b = (X@B[i])[0:3,0:3]
    t_a = (A[i]@X)[0:3,3]
    t_b = (X@B[i])[0:3,3]
    R_e.append(numpy.linalg.norm(R_a@R_b.T - eye(3)))
    t_e.append(numpy.linalg.norm(t_a - t_b))
    
print("Mean error in rotation: {0}".format(numpy.mean(R_e)))
print("Mean error in translation: {0}".format(numpy.mean(t_e)))
