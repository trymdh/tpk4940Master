import numpy as np 


K = np.array([[1,2,3],[4,5,np.nan],[np.nan,8,9]])

for i in K:
    for j in i:
        if j == np.nan:
            j = np.nan_to_num(j)

print(np.nan_to_num(K))