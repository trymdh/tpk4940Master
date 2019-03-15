# tpk4940Master
This repository is a work in progress, and is a collection of code for my master thesis at NTNU about development and performance evaluation of a machine vision system for multi-pass robotic welding of large subsea structures.

The machine vision system is composed of an industrial line laser and an AT C4 highspeed CMOS camera that uses the GiGE-standard. Automation Technologies have provided me with their SDK and standalone Python (3.5.4) for camera control. 

per. 10.03.2019
 - The repository mainly contains methods and functions for camera control and image processing via Python.




## Introduction to CVB
Common Vision Blox is a set of programming libraries aimed at image processing professionals. 
It has particular strengths in acquistion, is hardware-independent, flexible and fast.

## Installation and setup of CVB and CVBpy 
1. Install Python via the newest Anaconda Version: https://www.anaconda.com/download
2. Install Visual Studio Code editor: https://code.visualstudio.com/
3. Install CVB: https://www.commonvisionblox.com/en/download-cvb-windows-32-bit-and-64-bit/
4. Install the CVB wrappers: https://forum.commonvisionblox.com/t/common-vision-blox-add-ons/570
5. Open the "Anaconda Prompt" and create a new virtual enviroment.
   - Open "Anaconda Prompt"
   - Type "conda create -n py36 python=3.6 anaconda"
6. Activate the virtual environment
   - Type "activate py36" in the Anaconda Prompt terminal
7. Change the prompt folder to %CVB%\Lib\Python and install the CVBpy-wrapper.
   - Type "cd %CVB%\Lib\Python"
   - Type "pip install cvb-1.0-cp36m-win_amd64.whl"
