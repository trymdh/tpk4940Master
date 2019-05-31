import os
import re
import random
import numpy as np
import glob
from QuaternionFunctions import shepperd, qprod
from matplotlib import pyplot as plt
from matplotlib.ticker import MaxNLocator,ScalarFormatter 
from UtilityFunctions import getUname, sortList
from MatrixFunctions import skew, unskew, logMatrix
np.set_printoptions(suppress=True)

def handEye(A,B,cnt):
    #Solving AX = BX, need n >= 2 pairs of transforms, i.e len(A) = len(B) >= 2
    #based on MATLAB code in Olav Vision Notes
    n = len(A)
    Ka = np.zeros((3,n)); Kb = np.zeros((3,n))
    for i in range(0,n-1):
        Ka[:,i] = logMatrix(A[i][0:3,0:3]).ravel()
        Kb[:,i] = logMatrix(B[i][0:3,0:3]).ravel()

    #print(Kb.reshape(cnt,3))
    #print(np.isnan(Kb))
    #Kb = np.nan_to_num(Kb)
    H = np.dot(Kb,np.transpose(Ka))
    
    #print(H)
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

    #k = np.sort(np.random.choice(Ts_CW.shape[0],n,replace = False))
    #print(k)
    A = [] 
    B = []
    #calulate different pairs of poses
    cnt = 0
    for i in range(0,n):
        for j in range(i+1,n):
            if j != i:
                #combiantions
                #print(i,j)
                cnt += 1
                A.append(np.dot(np.linalg.inv(Ts_OE[j]),Ts_OE[i]))
                B.append(np.dot(Ts_CW[j],np.linalg.inv(Ts_CW[i])))
    print("Number of permutations: {0}".format(cnt))
    return np.asarray(A),np.asarray(B),cnt

R_error = []
t_error = []
cnt_list = []
#poses from 1-18
n = 8
for i in range(3,n):
    A,B,cnt = get_A_B(i)
    Rx,tx = handEye(A,B,cnt) #Transform from end effector to camera frame
    X = np.eye(4)
    X[0:3,0:3],X[0:3,3] = np.around(Rx,decimals = 6), np.around(tx,decimals=3)
    print(X)
    np.save("X.npy",X)
    q_metric = []
    t_ex = []
    t_ey = []
    t_ez = []
    R_e = []
    for i in range(0,len(A)):
        AX, XB = np.dot(A[i],X), np.dot(X,B[i])
        R_a, R_b = A[i][0:3,0:3], B[i][0:3,0:3]
        R_e.append(np.linalg.norm(np.eye(3) - np.dot(R_a,np.linalg.inv(R_b))))
        t_ax, t_xb = AX[0:3,3], XB[0:3,3]
        t_ex.append(np.linalg.norm(t_xb[0] - t_ax[0]))
        t_ey.append(np.linalg.norm(t_xb[1] - t_ax[1]))
        t_ez.append(np.linalg.norm(t_xb[2] - t_ax[2]))
        
        #calculate the deviation in rotation using unit quaternions
        q = shepperd(Rx@R_b@np.linalg.inv(Rx)) # R_a = Rx*R_b*(Rx)^-1
        q_a = shepperd(R_a)
        q_e = np.linalg.norm(1 - np.linalg.norm(qprod(q_a,qconj(q)))) #distance metric in a scalar approximate value
        q_metric.append(q_e)

    #print(np.mean(R_e))
    #print(t_e)
    print("Mean quaternion error in rotation is {0} units".format(np.mean(q_metric)))
    print("Mean error in translation [ex,ey,ez]: {0}mm, {1}mm, {2}mm".format(np.mean(t_ex),np.mean(t_ey),np.mean(t_ez)))
    R_error.append(np.mean(q_metric))
    t_error.append([np.mean(t_ex),np.mean(t_ey),np.mean(t_ez)])
    cnt_list.append(cnt)
t_error = np.asarray(t_error)
t = cnt_list
ax = plt.figure(1)
plt.subplot(211)
plt.title("Mean quaternion error in rotation")
plt.plot(t,R_error, label = "|1-|q_e||")
ax.gca().xaxis.set_major_locator(MaxNLocator(integer=True))
plt.legend()
plt.subplot(212)
plt.title("Mean Error in translation")
plt.plot(t,t_error[:,0], label = "e_x")
plt.plot(t,t_error[:,1], label = "e_y")
plt.plot(t,t_error[:,2], label = "e_z")
plt.ylabel("mm")
plt.xlabel("# permutations")
ax.gca().xaxis.set_major_locator(MaxNLocator(integer=True))
plt.legend()
plt.show()
