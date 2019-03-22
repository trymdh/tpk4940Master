import numpy as np

def weights(ReprojectionErrors):
    NumberOfImages = int(len(ReprojectionErrors[1,:])/2)
    temp_weights = {}
    weights = {}
    total_error = 0
    print(NumberOfImages)
    for i in range(NumberOfImages):
        sum_X = sum(abs(ReprojectionErrors[:,2*i]))
        sum_Y = sum(abs(ReprojectionErrors[:,2*i+1]))
        temp_weights[str(i+1)] = sum_X + sum_Y
        total_error += sum_X + sum_Y
    print(temp_weights)
    print(total_error)
    for key,val in temp_weights.items():
        weights[key] = total_error/val**2
    return weights

