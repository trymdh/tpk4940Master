import os
import msvcrt
import numpy as np
from PointArrayFunctions import homogenify
from PlaneFunctions import getCentroid3D, planeify, svd_AxB
from ErrorFunctions import getError, error_checker

def countInliers(error_vec, median_error,error_std,pointCloud,delta):
    i = 0
    cnt_in = 0
    cnt_out = 0
    Inliers = []
    Outliers = []
    for error in error_vec:
        if np.abs(error) < np.abs(median_error) - delta*np.abs(error_std):
            cnt_in += 1
            Inliers.append(pointCloud[i])
        else:
            cnt_out += 1
            Outliers.append(pointCloud[i])
        i += 1
    Inliers = np.asarray(Inliers)
    Outliers = np.asarray(Outliers)
    return Inliers,Outliers,cnt_in,cnt_out

def ransacPlane(pointCloud,delta,goal):
    bestFit = None
    bestErr = np.inf
    centroid = None
    best_cnt_in = 0
    goal_inlier = goal*len(pointCloud)
    N = np.inf
    ite = 0
    while ite < N:
        #sample 3 random points and estimate plane
        maybeIndex = np.random.choice(pointCloud.shape[0],3,replace = False)
        maybeInliers = pointCloud[maybeIndex,:]
        pI,c = svd_AxB(homogenify(maybeInliers))
        
        #calculate error and inlier threshold
        error_vec,median_error,error_std = getError(pointCloud,pI,c)
        #count inliers
        Inliers,Outliers, cnt_in,cnt_out = countInliers(error_vec, median_error,error_std,pointCloud,delta)
        if cnt_in >= goal_inlier:
            betterData = Inliers
            betterOut = Outliers
            betterFit,c = svd_AxB(homogenify(betterData))
            error = error_checker(betterFit,pointCloud)
            if (cnt_in > best_cnt_in) and error < bestErr:
                print(cnt_in,cnt_out)
                #update N
                w = cnt_in/len(pointCloud)
                N = np.log(1-0.99)/np.log(1-w**3)
                #Update best fit plane
                bestFit = betterFit
                best_cnt_in = cnt_in
                bestErr = error
                centroid = c
        ite += 1
    #print("Number of iterations:{0}".format(ite + 1))
    return bestFit,centroid,bestErr,betterData,betterOut

def ransacXn(uname,pointCloud,n):
    bestfit_folder = "C:/Users/" + str(uname) + "/OneDrive/tpk4940Master"
    os.chdir(bestfit_folder)
    #Running the ransac function n-times and return the best fit if the error is smaller than the "best-fit-to-date" error.
    bestFit = np.load('BestRansacPlane.npy')
    bestPlane,bestPlane_s = planeify(bestFit)
    bestErr = error_checker(bestPlane,pointCloud)
    for i in range(0,n):
        if msvcrt.kbhit():
            key = str(msvcrt.getch()).replace("b'","").replace("'","")
            if key == 'q':
                print("Loop exited")
                break
        ransac_fit,c,err,outliers = ransacPlane(pointCloud,1.5,0.1)
        print("Current error is:{0} \t Best error is: {1}".format(err,bestErr))
        if err < bestErr:
            bestFit = np.append(ransac_fit[0:3],c)
            bestPlane,bestPlane_s = planeify(bestFit)
            bestErr = err
            print("BestError changed to: {0}".format(bestErr))
            #np.save('BestRansacPlane.npy',bestFit)
    return bestPlane,c,bestErr
