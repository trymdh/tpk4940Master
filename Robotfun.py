from mpl_toolkits import mplot3d
import numpy as np 
import matplotlib.pyplot as plt

#Personal prefference...
pi = np.pi
c = np.cos
s = np.sin
transpose = np.transpose
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
	R = np.around(R,decimals = 4)
	return R

#Returns the rotation about the y-axis
def roty(angle):
	
	R = np.array([[c(angle), 0, s(angle)],[0, 1, 0],[-s(angle), 0, c(angle)]])
	
	#Evenly round to the given number of decimals.
	R = np.around(R,decimals = 4)
	return R

#Returns the rotation about the z-axis
def rotz(angle):

	R = np.array([[c(angle), -s(angle), 0],[s(angle), c(angle), 0], [0, 0, 1]])
	
	#Evenly round to the given number of decimals.
	R = np.around(R,decimals = 4)
	return R

def T_matrix(R,t):
	T = np.eye(4)
	T[0:3,0:3] = R
	T[0:3,3] = t
	
	return T

#HRotZ returns the homogeneous matrix-form of the rotation about the z-axis
def HRotZ(angle):
	R = rotz(angle)
	T = np.eye(4)
	T[0:3,0:3] = R
	return T

#HRotX returns the homogeneous matrix-form of the rotation about the z-axis
def HRotX(angle):
	R = rotx(angle)
	T = np.eye(4)
	T[0:3,0:3] = R
	return T

#TranslZ returns the homogeneous matrix-form of the translation along the z-axis
def TranslZ(z):
	T = np.eye(4)
	T[2,3] = z
	return T

#TranslX returns the homogeneous matrix-form of the translation along the x-axis
def TranslX(x):
	T = np.eye(4)
	T[0,3] = x
	return T

def fk_dh(dh_a, dh_alpha, dh_d, q_zero_offset, q):
    A1 = dh(dh_a[0], dh_alpha[0], dh_d[0], q[0] + q_zero_offset[0])
    A2 = dh(dh_a[1], dh_alpha[1], dh_d[1], q[1] + q_zero_offset[1])
    A3 = dh(dh_a[2], dh_alpha[2], dh_d[2], q[2] + q_zero_offset[2])
    A4 = dh(dh_a[3], dh_alpha[3], dh_d[3], q[3] + q_zero_offset[3])
    A5 = dh(dh_a[4], dh_alpha[4], dh_d[4], q[4] + q_zero_offset[4])
    A6 = dh(dh_a[5], dh_alpha[5], dh_d[5], q[5] + q_zero_offset[5])
    T01 = A1
    T02 = A1 @ A2
    T03 = A1 @ A2 @ A3
    T04 = A1 @ A2 @ A3 @ A4
    T05 = A1 @ A2 @ A3 @ A4 @ A5
    T06 = A1 @ A2 @ A3 @ A4 @ A5 @ A6
    return (T01, T02, T03, T04, T05, T06)

def shepperd(R):
#Quaternion from rotation matrix using Shepperd's algorithm,
#which is stable, does not lose significant precision and uses only one sqrt.
#J. Guidance and Control, 1 (1978) 223-224.
	q = transpose(np.zeros(4))

	z00 = R[0,0] + R[1,1] + R[2,2] # Trace of R
	z11 = R[0,0] + R[0,0] - z00
	z22 = R[1,1] + R[1,1] - z00
	z33 = R[2,2] + R[2,2] - z00
	#Find a large zii to avoid division by zero
	if z00 >= 0.5:
		w = sqrt(1.0 + z00)
		wInv = 1.0/w
		x = (R[2,1] - R[1,2])*wInv
		y = (R[0,2] - R[2,0])*wInv
		z = (R[1,0] - R[0,1])*wInv

	elif z11 >= 0.5:
		x = sqrt(1.0 + z11)
		xInv = 1.0/x
		w = (R[2,1] - R[1,2])*xInv
		y = (R[1,0] + R[0,1])*xInv
		z = (R[2,0] + R[0,2])*xInv
	elif z22 >= 0.5:
		y = sqrt(1.0 + z22)
		yInv = 1.0/y
		w = (R[0,2] - R[2,0])*yInv
		x = (R[1,0] + R[0,1])*yInv
		z = (R[2,1] + R[1,2])*yInv
	else:
		z = sqrt(1.0 + z33)
		zInv = 1.0/z
		w = (R[1,0] - R[0,1])*zInv
		x = (R[2,0] + R[0,2])*zInv
		y = (R[2,1] + R[1,2])*zInv

	eta = 0.5*w
	eps = 0.5*np.array([[x],[y],[z]])

	q[0] = eta
	q[1:4] = transpose(eps)
	q = np.around(q,decimals = 4)
	return q

def qprod(q1,q2):
	eta1 = q1[0] 
	eps1 = q1[1:4] 
	eta2 = q2[0] 
	eps2 = q2[1:4] 
	q = np.zeros(4)
	
	#formula (217) on page 47 in the notes.
	q[0] = eta1*eta2 - np.dot(eps1,eps2)
	q[1:4] =  eta1*eps2 + eta2*eps1 + np.cross(eps1,eps2)
	q = np.around(q,decimals = 4)
	return q

def quat2rot(q):
#Takes a quaternion and returns its corresponding rotation matrix
	n = q[0]
	e_x = q[1]
	e_y = q[2]
	e_z = q[3]
	R = np.eye(3)
	R[0,0:3] = np.array([2*(n**2 + e_x**2)-1,2*(e_x*e_y - n*e_z),2*(e_x*e_z + n*e_y)])
	R[1,0:3] = np.array([2*(e_x*e_y + n*e_z),2*(n**2 + e_y**2)-1,2*(e_y*e_z - n*e_x)])
	R[2,0:3] = np.array([2*(e_x*e_z - n*e_y),2*(e_y*e_z + n*e_x),2*(n**2 + e_z**2)-1])
	return R

def quatdata2Rotdata(q_unit_interpolated_data):
	R =[]
	for q in q_unit_interpolated_data:
		R.append(quat2rot(q))
	return np.array(R)

def T_matrix_interp(R_interp,p_interp):
	T = []
	for i in range(0,R_interp.shape[0]):
		T.append(T_matrix(R_interp[i],p_interp[i]))
	return np.array(T)

def slerp(v0, v1, t_array):
#Slerp is shorthand for spherical linear interpolation, 
#introduced by Ken Shoemake in the context of quaternion interpolation for the purpose of animating 3D rotation
#>>> slerp(q0,q1,np.arange(0,1,0.001))

    t_array = np.array(t_array)
    dot = np.sum(v0*v1)

    if (dot < 0.0):
        v1 = -v1
        dot = -dot
    
    DOT_THRESHOLD = 0.9995
    if (dot > DOT_THRESHOLD):
        result = v0[np.newaxis,:] + t_array[:,np.newaxis]*(v1 - v0)[np.newaxis,:]
        result = result/np.linalg.norm(result)
        return result
    
    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)

    theta = theta_0*t_array
    sin_theta = np.sin(theta)
    
    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return (s0[:,np.newaxis] * v0[np.newaxis,:]) + (s1[:,np.newaxis] * v1[np.newaxis,:])

def drawFrame(R,t):

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim((-0.5,0.5))
    ax.set_ylim((-1,0))
    ax.set_zlim(-0.5,0.5)
    
    i = 0
    for r in R:
        origin = [t[i][0],t[i][1],t[i][2]]
        x = r[0:3,0]
        y = r[0:3,1]
        z = r[0:3,2]
        f = 10
        x_ax = ax.quiver(origin[0],origin[1],origin[2],x[0]/f,x[1]/f,x[2]/f, color = 'red', arrow_length_ratio = 0.05)
        y_ax = ax.quiver(origin[0],origin[1],origin[2],y[0]/f,y[1]/f,y[2]/f, color = 'blue', arrow_length_ratio = 0.05)
        z_ax = ax.quiver(origin[0],origin[1],origin[2],z[0]/f,z[1]/f,z[2]/f, color = 'green',arrow_length_ratio = 0.05)
        i = i + 1
    plt.show()

    return 0

def plotIntdata(p_interp,startpos,endpos,origin):
	#p_interp.shape(iterations,3)
	xdata = p_interp[:,0]
	ydata = p_interp[:,1]
	zdata = p_interp[:,2]

	fig = plt.figure()
	ax = plt.axes(projection='3d')
	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')
	
	#plots a quiver between a reference frame (0,0,0) and the endpoints a and b
	ax.quiver(origin[0],origin[1],origin[2],startpos[0],startpos[1],startpos[2], color = 'red',arrow_length_ratio = 0.05)
	ax.quiver(origin[0],origin[1],origin[2],endpos[0],endpos[1],endpos[2], color = 'blue', arrow_length_ratio = 0.05)

	#plots the interpolated points
	ax.scatter3D(xdata,ydata,zdata)
	plt.show()

def lin_int_pos(startpos,endpos,iterations):
	v = endpos-startpos
	p_interp = np.zeros((iterations,3))
	for i in range(0,iterations):
		p_interp[i,0:3]= startpos + (v/iterations)*i

	return p_interp

def ik_analytical(T, shoulder=1, elbow=1, wrist=1):
    a = [0.00000, -0.42500, -0.39225,  0.00000,  0.00000,  0.0000]
    d = [0.089159,  0.00000,  0.00000,  0.10915,  0.09465,  0.0823]
    alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0. ]
    q_zero_offset = [0., 0., 0., 0., 0., 0.]
    
    t_0e = T[0:3,3]
    a_e = T[0:3,2]
    s_e = T[0:3,1]
    n_e = T[0:3,0]
    
    q = np.zeros(6)
    p_e = np.array(T[0:3,3])
    a_e = np.array(T[0:3,2])
    
    #eq. 414 in notes
    p_5 = p_e-d[5]*a_e
    
    #theta1
    q[0] = np.arctan2(shoulder*p_5[1],shoulder*p_5[0]) + shoulder*np.arctan2(d[3],np.sqrt(p_5[0]**2+p_5[1]**2 - d[3]**2))
    
    x_1 = np.array([np.cos(q[0]),np.sin(q[0]),0])
    y_1 = np.array([0,0,1])
    z_1 = np.array([np.sin(q[0]),-np.cos(q[0]),0])
    
    z_4_temp = np.cross(z_1,a_e)
    z_4 = wrist*shoulder*(z_4_temp/np.linalg.norm(z_4_temp))

    x_4 = np.cross(z_1,z_4)
    y_4 = z_1

    #theta5 eq.417
    q[4] = np.arctan2(np.dot(-a_e,x_4),np.dot(a_e,y_4))
    #theta6 eq.418
    q[5] = np.arctan2(np.dot(-z_4,n_e),np.dot(-z_4,s_e))

    p_3 = p_5 - d[4]*z_4 - d[3]*z_1
    p_h = shoulder*np.sqrt(p_3[0]**2 + p_3[1]**2)
    p_v = p_3[2]-d[0]

    #eq. 420-421
    c_3 = (p_h**2 + p_v**2 - a[1]**2 - a[2]**2)/(2*a[1]*a[2])
    s_3 = elbow*np.sqrt(1 -c_3**2)

    #theta3 eq.422
    q[2] = np.arctan2(s_3,c_3)

    #eq.423-424
    c_2 = (p_h*(a[1] + a[2]*c_3) + p_v*a[2]*s_3)/(p_h**2 + p_v**2)
    s_2 = (p_v*(a[1] + a[2]*c_3) - p_h*a[2]*s_3)/(p_h**2 + p_v**2)

    #theta2 eq.425
    q[1] =  np.arctan2(s_2,c_2)

    #eq.426-427
    s_234 = np.dot(z_4,x_1)
    c_234 = np.dot(-z_4,y_1)
    theta234 = np.arctan2(s_234,c_234)

    #theta4 eq.428 
    q[3] = theta234 - (q[1] + q[2])
 
    #In case we are "overrunning" our joints we need to reset the rotations
    #of the joints to a value below the joint limits
    for i in range(0,6):
                while (q[i] > np.pi) or (q[i] < -np.pi):
                    if (q[i] > np.pi):
                        q[i] = q[i] - np.pi
                    elif (q[i] < -np.pi):
                        q[i] = q[i] + np.pi
    return q

def dh(a, alpha, d, theta):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    return np.array([[ct, -st * ca,  st * sa, a * ct],
                     [st,  ct * ca, -ct * sa, a * st],
                     [0.0,      sa,       ca, d],
                     [0.0,     0.0,      0.0, 1.0]])

def fk(q):
    a = [0.00000, -0.42500, -0.39225,  0.00000,  0.00000,  0.0000]
    d = [0.089159,  0.00000,  0.00000,  0.10915,  0.09465,  0.0823]
    alpha = [ 1.570796327, 0, 0, 1.570796327, -1.570796327, 0 ]
    q_zero_offset = [0, 0, 0, 0, 0, 0]
    return fk_dh(a, alpha, d, q_zero_offset, q)

class LSPBError(Exception):
    pass

def lspb_trajectory(current_position, target_position,
                    duration_in_seconds, cruise_velocity=None):

    q0 = current_position
    q1 = target_position
    tf = duration_in_seconds
    V = cruise_velocity

    ts = np.linspace(0.0, tf)

    if V is None:
        V = (q1-q0)/tf * 1.5
    else:
        V = np.abs(V) * np.sign(q1-q0)
        if np.abs(V) < np.abs(q1-q0)/tf:
            raise LSPBError('V too small')
        elif np.abs(V) > 2 * np.abs(q1-q0)/tf:
            raise LSPBError('V too big')

    if np.allclose(q0, q1):
        s = np.ones(len(ts)) * q0
        sd = np.zeros(len(ts))
        sdd = np.zeros(len(ts))
        return (s, sd, sdd)

    tb = (q0 - q1 + V*tf)/V
    a = V/tb

    p = []
    pd = []
    pdd = []

    for tt in ts:
        if tt <= tb:
            # Initial blend
            p.append(q0 + a/2*tt*tt)
            pd.append(a*tt)
            pdd.append(a)
        elif tt <= (tf - tb):
            p.append((q1+q0-V*tf)/2 + V*tt)
            pd.append(V)
            pdd.append(0.0)
        else:
            p.append(q1 - (a/2*tf*tf) + a*tf*tt - a/2*tt*tt)
            pd.append(a*tf - a*tt)
            pdd.append(-a)

    return (p, pd, pdd, ts)

def plot_trajectory(trajectory, title=''):
    qs, dqs, ddqs, ts = trajectory
    f, axarr = plt.subplots(3, sharex=True)
    axarr[0].plot(ts, qs, color='red')
    axarr[0].set_title(title)
    axarr[0].set_ylabel('position')
    axarr[1].plot(ts, dqs, color='green')
    axarr[1].set_ylabel('velocity')
    axarr[2].plot(ts, ddqs, color='blue')
    axarr[2].set_ylabel('acceleration')
    axarr[2].set_xlabel('time [s]')
    plt.show()