from AT_cx_functions import*
import numpy as np 
from matplotlib import pyplot as plt 
import os 
import glob 

uname = getUname()
endEffPosPath = "C:/Users/" + str(uname) + "/OneDrive/tpk4940Master/EndeffPos20"
os.chdir(endEffPosPath)
Ts = sortList(glob.glob(os.getcwd().replace("\\","/") + "/*.npy"))
i = 0
for T in Ts:
    pose = np.load(T)
    print("Pose number {0}: \n {1} \n".format(i,pose))
    i += 1