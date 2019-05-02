The initial configuration of the camera work distance and focus is done using the CVB Management Console.

Camera calibration:
1. Open laserandimagesnap.py 
2. A default location where the calibration images and laserline.npy images are saved, is already spesifed but can be changed in line 16.
3. Run it as many times as needed to get a good calibration, i.e n >= 20
4. Open MATLAB and use the camera calibration plugin to calibrate the camera 
5. Export the calibration parameters using writeCaliParam.m, if the estimation errors is needed use writeEstimationError.m

Laser plane calibration:
1. Import the camera calibration parameters using the loadCaliParam function.
2. Import the laser.npy files located in the calibration image folder.
3. The extractPoints() function returns the laserline points in camera coordinates.
4. The extracted points are then run through a ransac algorithm that estimates the laser plane. 
5. The ransacXn() function runs the ransac algorithm n-times and checks the result of each iteration up against the "best-to-date-plane", and returns the plane with the smallest error. This is done because the ransac algorithm does not always return the best fit, due to the random nature of the algorithm.









