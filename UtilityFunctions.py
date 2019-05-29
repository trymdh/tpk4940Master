import re
import os
import numpy as np

def getUname():
    #get the system username, usefull when working on different computers.
    uname = os.getlogin()
    if uname == "trymdh":
        uname = uname + ".WIN-NTNU-NO"
    return uname

def getCurrentWD():
    #Get path to current work directory with correct backslash format used in glob.
    wd = os.getcwd().replace("\\", "/")
    return wd

def sortList(unsortedList):
    #sorts a list in alphanumeric order
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)',key)]
    return sorted(unsortedList,key = alphanum_key)

