# tpk4940Master
This repository shows how to setup the Stemmer Imaging CVB library and its CVBpy-wrapper for Python 3.6 on Windows 64 bit.

## Introduction to CVB
Common Vision Blox is a set of programming libraries aimed at image processing professionals. 
It has particular strengths in acquistion, is hardware-independent, flexible and fast.

## Installation and setup of CVB and CVBpy 
1. Install Python via the newest Anaconda Version: https://www.anaconda.com/download
2. Install Visual Studio Code editor: https://code.visualstudio.com/
3. Install the CVB wrappers: https://forum.commonvisionblox.com/t/common-vision-blox-add-ons/570
4. Open the "Anaconda Prompt" and create a new virtual enviroment.
   - Open "Anaconda Prompt"
   - Type "conda create -n py36 python=3.6 anaconda"
5. Activate the virtual environment
   - Type "activate py36" in the Anaconda Prompt terminal
6. Change the prompt folder to %CVB%\Lib\Python and install the CVBpy-wrapper.
   - Type "cd %CVB%\Lib\Python"
   - Type "pip install cvb-1.0-cp36m-win_amd64.whl"
