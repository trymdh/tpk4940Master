import numpy as np
#Personal prefference...
pi = np.pi
c = np.cos
s = np.sin
transpose = np.transpose
inv = np.linalg.inv
sqrt = np.sqrt

def vec(x,y,z):
    return np.array([x,y,z])

def cvec(x,y,z):
    return np.array([vec(x,y,z)]).T

def skew(k):
    return np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])

#Returns the rotation about the x-axis
def rotx(angle):
	R = np.array([[1, 0, 0],[0, c(angle), -s(angle)],[0, s(angle), c(angle)]])
	#Evenly round to the given number of decimals.
	return R

#Returns the rotation about the y-axis
def roty(angle):
	R = np.array([[c(angle), 0, s(angle)],[0, 1, 0],[-s(angle), 0, c(angle)]])
	#Evenly round to the given number of decimals.
	return R

#Returns the rotation about the z-axis
def rotz(angle):
	R = np.array([[c(angle), -s(angle), 0],[s(angle), c(angle), 0], [0, 0, 1]])
	return R

def T_matrix(R,t):
	T = np.eye(4)
	T[0:3,0:3] = R
	T[0:3,3] = t
	return T

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