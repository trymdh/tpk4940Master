def planeify(vector_plane): #Assumes form [nx,ny,nz,cx,cy,cz]
    #[A,B,C,D]
    plane = [vector_plane[0],vector_plane[1],vector_plane[2],-vector_plane[0]*(vector_plane[3])-vector_plane[1]*(vector_plane[4])-vector_plane[2]*(vector_plane[5])] 
    #Or on form [Ax + By + D] = z
    plane_s = [-plane[0]/plane[2],-plane[1]/plane[2],-plane[3]/plane[2]]
    return plane,plane_s

#def normifyPlane() #Assumes form Ax + By + D = z