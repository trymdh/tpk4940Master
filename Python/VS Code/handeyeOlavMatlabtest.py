import numpy as np
from AT_cx_functions import*
from Robotfun import rotx,roty,rotz

#Unknown transformation
X = np.eye(4)
X[0:3,0:3] = rotx(np.pi/6)
X[0:3,3] = np.array([1,2,1])
#input: Point Data
B1 = np.eye(4)
B1[0:3,0:3] = rotz(np.pi/8)
B1[0:3,3] = np.array([0,0,1])
B2 = np.eye(4)
B2[0:3,0:3] = roty(np.pi/7)
B2[0:3,3] = np.array([0,1,1])
B3 = np.eye(4)
B3[0:3,0:3] = roty(np.pi/6)@rotz(-np.pi/6)@rotx(np.pi/9)
B3[0:3,3] = np.array([0,1,1])
B = [B1,B2,B3]

A1 = X@B1@np.linalg.inv(X)
A2 = X@B2@np.linalg.inv(X)
A3 = X@B3@np.linalg.inv(X)

A = [A1,A2,A3]
A = np.asarray(A)

X = handEye(A,B)