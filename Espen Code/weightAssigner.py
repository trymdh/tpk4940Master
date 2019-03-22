import numpy as np

def weights(Errors1,Errors2,mode):
    if mode == 'Repr': #This mode uses the reprojection errors as quantitative measurement as according to tot_error/error^2 which yields high cost to low error points
        NumberOfImages = int(len(Errors1[1,:])/2)
        temp_weights = {}
        weights = {}
        total_error = 0
        print(NumberOfImages)
        for i in range(NumberOfImages):
            sum_X = sum(abs(Errors1[:,2*i]))
            sum_Y = sum(abs(Errors1[:,2*i+1]))
            temp_weights[str(i+1)] = sum_X + sum_Y
            total_error += sum_X + sum_Y
        for key,val in temp_weights.items():
            weights[key] = total_error/val**2
    
    if mode == 'Vecs': #This mode penalizes both error in rotation and translation, in which the latter yield bigger influence (as they tend to be larger)
        rot_errors = 0
        trans_errors = 0
        NumberOfImages = int(len(Errors1[:,0]))
        temp_weights = {}
        weights = {}
        for i in range(len(Errors2[:,0])):
            error = np.sqrt(Errors1[i,0]**2 + Errors1[i,1]**2 + Errors1[i,2]**2) 
            temp_weights[str(i+1)] = error
            rot_errors += error

        for i in range(len(Errors2[:,0])):   
            t_error = Errors2[i,0]**2 + Errors2[i,1]**2 + Errors2[i,2]**2
            trans_errors += t_error
            temp_weights[str(i+1)] += t_error
        
        for key,val in temp_weights.items():
            weights[key] = rot_errors + trans_errors / (val)**2
    return weights

