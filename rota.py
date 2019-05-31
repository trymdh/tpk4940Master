import numpy as np 
from MatrixFunctions import rotx,roty,rotz,logMatrix
pi = np.pi
R = np.load("X.npy")[:3,:3]
R_i = rotz(pi)@roty(-pi/2)

error = np.linalg.norm(logMatrix(R_i@R.T))
print(error)