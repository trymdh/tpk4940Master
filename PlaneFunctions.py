import numpy as np

def estimatePlane(points):
    #This function estimates a plane from three points and return the plane parameters
    #if the condition for point to be in the plane is satisfied.
    p_1 = points[0]; p_2 = points[1]; p_3 = points[2]
    #compute two vectors in the plane
    v1 = p_1 - p_3; v2 = p_2 - p_3
    #centroid
    cent = getCentroid3D(points)
    #The plane normal is then the cross product of these two vectors, n = [a,b,c]
    n = np.cross(v1,v2)
    #distance from the plane to the origin
    d = -np.dot(p_3,np.cross(p_1,p_2))*np.linalg.norm(n)
    #Plane, pI = [n,d] = [a,b,c,d]
    pI = np.append(n,d)
    #p_h = [x,y,z,1]
    p_1_h = np.append(p_1,1); p_2_h = np.append(p_2,1); p_3_h = np.append(p_3,1)
    #Criteria for a point to be in the plane is x_h . Pi ~ 0 is satisfied.
    cri1 = np.dot(p_1_h,pI); cri2 = np.dot(p_2_h,pI); cri3 = np.dot(p_3_h,pI)
    conditions = [np.around(cri1,decimals=7) == 0,np.around(cri3,decimals=7) == 0,np.around(cri3,decimals=7) == 0]

    if all(conditions):
        return pI,cent
    else:
        return None

def lsPlane(pointCloud):
    """
    Code below is based on the user "BEN" from 
    https://stackoverflow.com/questions/1400213/3d-least-squares-plane
    """
    x = pointCloud[:,0]
    y = pointCloud[:,1]
    z = pointCloud[:,2]
    c = getCentroid3D(pointCloud)

    A = []
    b = []
    for i in range(len(x)):
        A.append([x[i], y[i], 1])
        b.append(z[i])
    A = np.matrix(A)
    b = np.matrix(b).T

    fit = (A.T @ A).I @ A.T @ b 
    error = b - A @ fit

    v1 = np.array([1,0,fit[0]])
    v2 = np.array([0,1,fit[1]])
    
    #planenormal
    n = np.cross(v1,v2)
    n /= np.linalg.norm(n)

    return np.append(n,c)

def svd_AxB(pointCloud):
    u,s,vh = np.linalg.svd(pointCloud)
    v = vh.conj().T
    x = v[:,-1]
    x = x.T.reshape(-1)
    c = getCentroid3D(pointCloud)
    n = x[0:3]
    d = x[3]/np.linalg.norm(n)
    
    #[A,B,C,D]
    pI = x*d
    return pI,c

def getCentroid3D(pointCloud):
    #this function returns the centroid
    xs = pointCloud[:,0]; ys = pointCloud[:,1]; zs = pointCloud[:,2]
    x_av = np.average(xs); y_av = np.average(ys); z_av = np.average(zs)
    return np.array([x_av,y_av,z_av])

def planeify(vector_plane): 
    #Assumes form [n, p_0]
    #a*x_0 + b*y_0 + c*z_0 + d = 0
    #-> d = - (a*x_0 + b*y_0 + c*z_0)
    D = -(vector_plane[0]*(vector_plane[3])+vector_plane[1]*(vector_plane[4])+vector_plane[2]*(vector_plane[5]))
    #[A,B,C,D]
    plane = np.asarray([vector_plane[0],vector_plane[1],vector_plane[2],D])
    #Or on form [Ax + By + D] = z
    plane_s = np.asarray([-plane[0]/plane[2],-plane[1]/plane[2],-plane[3]/plane[2]])
    return plane,plane_s

def getPlaneData(pI,ax):
    #This function takes the plane parameters and the matplotlib plot object as input 
    #and returns the data values needed to plot a wireframe using plt.subplot.plot_wireframe()
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),np.arange(ylim[0], ylim[1]))
    Z = np.zeros(X.shape)
    for r in range(X.shape[0]):
        for c in range(X.shape[1]):
            #z = a*X + b*Y + d 
            Z[r,c] = -(pI[0] * X[r,c] + pI[1] * Y[r,c] + pI[3])*1./pI[2]
    return X,Y,Z

def plotPlane(planeFit,ax,clr,alp):
    X,Y,Z = getPlaneData(planeFit,ax)
    ax.plot_wireframe(X,Y,Z, color = clr, alpha = alp)
    