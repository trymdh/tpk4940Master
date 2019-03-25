from AT_cx_functions import*
import numpy as np 


a = np.random.rand(3,3)
a = homogenify(a)


x = svd_AxB(a)
print(a[0])
print(np.around(a[0]@x))