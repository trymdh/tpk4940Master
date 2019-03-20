from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import os

#DETTE BLIR EN FUNKSJON AV ROT,VEC,K-MATRISE,DIST OG PIKSELKOORD
#Følgende er hardcodet per nå : RotM, tVec og K-matrise
#Funksjonen tar inn Rotasjonsmatrise, tVec, K-matrise, distortionkoeffs (tas ikke høyde for per nå) i tillegg til enten directorien til 
#pikselcoord, eller hele preloaded pixelkoords.
#Funksjonen returnerer en array med XYZ-koordinater 
os.chdir('C:/Users/Trym/OneDrive/Master/VS Code/laserimage')
pix = np.load('pixcoord_4.npy')

#RotM and tVec is directly read from Matlab, corresponding to the 4th image
RotM = np.array([[0.9808 , 0.0352 , -0.1919],
                [-0.0256 , 0.9983 , 0.0523],
                [0.1934 , -0.0464 , 0.98]])
tVec = np.array([-140.4751 , -85.8216 , 1058.3])                

#Define Homogenous matrix (Blir ikke direkte brukt, men kan være kjekk å ha)
Hom_M = np.identity(4)
Hom_M[0:3,0:3] = RotM
Hom_M[0:3,3] = tVec

#Hardcoded K-matrix read from Matlab , this K_T is to be 
K_T = np.array([[3070.7,0,0],
            [0,3066.9,0],
            [1019.7,564.2325,1]])
K = np.transpose(K_T)
K_inv = np.linalg.inv(K)

extracted_points = np.array([])
#extracted_points = np.load("test_extraction.npy") #Kommenter ut forrige linje om denne skal brukes

#Define range for pixel calculation , ingen spesiell logikk her annet at jeg prøvde å unngå finger-piksler
pix_from = 480
pix_to = 1650
step = 5

for pix_index in range(pix_from,pix_to,step):
    if pix[pix_index,1] != 0:
        coord = pix[pix_index,1]   #Correct for distorition later                   
        pixel_coord = np.array([pix_index,coord,1])
        #Camera frame coordinate using s~ = K_inv * p
        cf_c = K_inv@pixel_coord #Transpose may be removed later
        cf_c_norm = cf_c / cf_c[2] #Maybe use linalg instead?

        #Z-axis of chessboard, normalen til selve chessboardet vil være z-aksen, som hentes ut av tredje rad i RotM
        n_vec = RotM[2,:] 


        #Calculation of line/plane intersection, samme metode som blir ellers brukt for triangulering
        p_0 = tVec #Punkt på plan, vil naturlig nok bli tVec i denne sammenheng
        n = n_vec #Normalen til planet
        l_0 = np.array([0,0,0]) #Et punkt på linjen, vil være senter av kamera-cooord
        l = cf_c_norm #Retningen på linjen
        num = np.dot((p_0 - l_0),n)
        deno = np.dot(l,n)
        d = num/deno
        
        cf_fullcoord = np.array([cf_c_norm * d]) # + l_0 , which is a zero-vector
        
        #if cf_fullcoord[1] == -4.09825224e+00:
         #   print(pixel_coord)

        extracted_points = np.append(extracted_points,cf_fullcoord)
extracted_points = np.reshape(extracted_points,(-1,3))
print(extracted_points)

#np.save('test_extraction.npy',extracted_points) # Denne kan uncommentes sammen med linje 33, dersom man vil hente fra flere bilder.
#Tanken var en enkel plass å lagre pointsene fra de forskjellige bildene, men er nokk HELT sikkert bedre å lage en separat fil for hvert av bildene, og
#og heller hente dem sammen senere for å generere en point-cloud


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x = extracted_points[:,0]
y = extracted_points[:,1]
z = extracted_points[:,2]

ax.scatter(x,y,z, c='r', marker='o')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_zlim(1200,0)
plt.show()

plt.plot(pix[:,1])
plt.gca().invert_yaxis()
plt.show()


