import numpy as np
from numpy import pi,cos,sin,tan,dot
import os
import glob
import re
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

def dh(a, alpha, d, theta):
    #Computes the transformation between link "i-1" and link "i", 
    #that is T^(i-1)_i
    ct = cos(theta)
    st = sin(theta)
    ca = cos(alpha)
    sa = sin(alpha)
    return np.array([[ct, -st * ca,  st * sa, a * ct],
                     [st,  ct * ca, -ct * sa, a * st],
                     [0,      sa,       ca, d],
                     [0,     0,      0,   1]])

def fk_dh(dh_a, dh_alpha, dh_d, q_zero_offset, q):
    A1 = dh(dh_a[0], dh_alpha[0], dh_d[0], q[0] + q_zero_offset[0])
    A2 = dh(dh_a[1], dh_alpha[1], dh_d[1], q[1] + q_zero_offset[1])
    A3 = dh(dh_a[2], dh_alpha[2], dh_d[2], q[2] + q_zero_offset[2])
    A4 = dh(dh_a[3], dh_alpha[3], dh_d[3], q[3] + q_zero_offset[3])
    A5 = dh(dh_a[4], dh_alpha[4], dh_d[4], q[4] + q_zero_offset[4])
    A6 = dh(dh_a[5], dh_alpha[5], dh_d[5], q[5] + q_zero_offset[5])
    T01 = A1
    T02 = dot(T01,A2)
    T03 = dot(T02,A3)
    T04 = dot(T03,A4)
    T05 = dot(T04,A5)
    T06 = dot(T05,A6)
    return T06

def fk(q):
    #DH parameters KUKA KR16
    q_zero_offset = [0, 0, -pi/2, 0, 0, 0]
    d =             [0.675,  0, 0,  0.670, 0,  0.115]
    a =             [0.260, 0.680, -0.035,  0,  0,  0]
    alpha =         [-pi/2, 0, -pi/2, pi/2, -pi/2, 0 ]
    

    return fk_dh(a, alpha, d, q_zero_offset, q)

uname = getUname()
os.chdir("C:/Users/" + uname +"/Onedrive/tpk4940Master/Camera calibration May/EndEffectorPositions/q_pose")
q_pose_list = sortList(glob.glob(os.getcwd().replace("\\","/") + "/*.npy"))
os.chdir("C:/Users/" + uname +"/Onedrive/tpk4940Master/Camera calibration May/EndEffectorPositions/T_pose")
T_pose_list = sortList(glob.glob(os.getcwd().replace("\\","/") + "/*.npy"))
T_dh = []
for fname in q_pose_list:
    q = np.load(fname)
    T = fk(q)
    T[:3,3] = T[:3,3]*1000
    T_dh.append(T)

T_dh = np.asarray(T_dh)

i = 0
for fname in T_pose_list:
    T = np.load(fname)
    print(T)
    print(T_dh[i])
    i += 1
