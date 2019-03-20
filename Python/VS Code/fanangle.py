import numpy as np 

def fanAngle(D,L):
    """
    Calculates the fan angle when the work distance and laser line length is given.
    """
    q = 2*np.arccos(D/np.sqrt(D**2 + (L**2)/4))
    q = q*(180/np.pi)
    return np.around(q,decimals=0)

def d_and_L(q):
    """
    This function finds all the combinations of work distances and lengths of projection consistent with a q-degree fan angle.
    """
    Ds = []
    Ls = []
    for D in range(100): #D in cm
        for L in range(100): #L in cm
            Q = fanAngle(D,L)
            if Q == q:
                Ds.append(D)
                Ls.append(L)
    return Ds,Ls

def workDistance(q,L):
    """
    This function returns the workdistance when we know the desired fan angle and length of laser line projection
    """
    Ds = []
    for D in range(100): #D in cm
        Q = fanAngle(D,L)
        if Q == q:
            Ds.append(D)
    if len(Ds) == 1:
        return Ds[0]
    else:
        return Ds

D = workDistance(45,40)
print(D)

#Ds,Ls = d_and_L(45)
#print(Ds)
#print(Ls)





