import numpy as np

# Initialize arrays
p1 = np.array([1,2])
p2 = np.array([2,3])
p3 = np.array([3,4])

def point(p):
    return np.array([p[0],p[1],1.0])

def colinearity_check_float(p1,p2,p3, epsilon=1e-2):
    matrix = np.vstack((point(p1),point(p2),point(p3)))
    det = np.linalg.det(matrix)
    if det < epsilon:
        return True
    return False

def collinearity_int(p1, p2, p3): 
    collinear = False
    det = p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1])
    if det == 0:
        collinear = True
    return collinear

import time
t1 = time.clock()
collinear = colinearity_check_float(p1, p2, p3)
t_3D = time.clock() - t1


t2 = time.clock()
colinear = collinearity_int(p1,p2,p3)
t_2D = time.clock() - t2
print(t_3D/t_2D)
