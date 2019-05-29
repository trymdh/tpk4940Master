import os
import msvcrt
import numpy as np
from MatrixFunctions import homogenify
from PlaneFunctions import getCentroid3D, planeify, svd_AxB
from ErrorFunctions import getError, error_checker

def countInliers(error_vec, median_error,error_std,pointCloud):
    i = 0
    cnt_in = 0
    Inliers = []
    Outliers = []
    for error in error_vec:
        if np.abs(error) < np.abs(median_error) + 0.5*np.abs(error_std):
        #if np.abs(error) < 0.2:
            cnt_in += 1
            Inliers.append(pointCloud[i])
        else:
            Outliers.append(pointCloud[i])
        i += 1
    Inliers = np.asarray(Inliers)
    Outliers = np.asarray(Outliers)
    return Inliers,Outliers

def ransacPlane(pointCloud):
    bestFit = None
    bestErr = np.inf
    centroid = None
    best_cnt_in = 0
    goal_inlier = 0.7*len(pointCloud)
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
        Inliers, outliers = countInliers(error_vec, median_error,error_std,pointCloud)
        cnt_in = len(Inliers)
        if cnt_in >= goal_inlier:
            betterData = Inliers
            BetterOutliers = outliers
            #The new dataset contains few outliers => use LS to estimate plane
            #betterFit,betterRes = lsPlane(betterData)
            betterFit,c = svd_AxB(homogenify(betterData))
            error = error_checker(betterFit,pointCloud)
            if (cnt_in >= best_cnt_in) and (error < bestErr):
                #update N
                w = cnt_in/len(pointCloud)
                N = np.log(1-0.99)/np.log(1-w**3)
                #Update best fit plane
                bestFit = betterFit
                best_cnt_in = cnt_in
                bestErr = error
                centroid = c
                #print("\nIteration {0}".format(ite + 1))
                #print("Inlier count: {0} / {1}".format(best_cnt_in,len(pointCloud)))
                #print ("{0}x + {1}y + {2}z + {3}".format(bestFit[0], bestFit[1], bestFit[2],bestFit[3]))
                #print ("Error:")
                #print(bestErr)
        
        ite += 1
    #print("Number of iterations:{0}".format(ite + 1))
    return bestFit,centroid,bestErr,BetterOutliers

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
        ransac_fit,c,err,outliers = ransacPlane(pointCloud)
        print("Current error is:{0} \t Best error is: {1}".format(err,bestErr))
        if err < bestErr:
            bestFit = np.append(ransac_fit[0:3],c)
            bestPlane,bestPlane_s = planeify(bestFit)
            bestErr = err
            print("BestError changed to: {0}".format(bestErr))
            #np.save('BestRansacPlane.npy',bestFit)
    return bestPlane,c,bestErr