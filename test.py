import os
import re
import random
import numpy as np
import glob
np.set_printoptions(suppress=True)
pi = np.pi
c = np.cos
s = np.sin
transpose = np.transpose
sqrt = np.sqrt

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
    
uname = getUname()
os.chdir("C:/Users/"+uname + "/Onedrive/tpk4940Master")
plane = np.load("BestRansacPlane.npy")
print(plane)
print(10*plane[0:3])
print(2.7*10**2)