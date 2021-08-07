import numpy as np 
import matplotlib.pyplot as plt 
plt.rcParams['figure.figsize'] =  [12,12]


#######################################
#######################################
# Custom Implementation of Bresenham #

def bresenham(p1,p2):
    x1,y1 = p1
    x2,y2 = p2
    cells = []
    if x1<x2 and y1<y2:
        m = (y2-y1)/(x2-x1)
        while x1<x2:
            cells.append((x1,y1))
            if (x1 +1)*m >(y1+1):
                y1 += 1
            elif (x1+1)*m == y1+1:
                x1 += 1
                y1 += 1
            else:
                x1 += 1
        return cells

# (x + 1) * m < y + 1

p1 = (0, 0)
p2 = (7, 5)

cells = bresenham(p1, p2)
# print(cells)

plt.plot([p1[0], p2[0]], [p1[1], p2[1]])


for q in cells:
    plt.plot([q[0], q[0]+1], [q[1], q[1]], 'k')
    plt.plot([q[0], q[0]+1], [q[1]+1, q[1]+1], 'k')
    plt.plot([q[0], q[0]], [q[1],q[1]+1], 'k')
    plt.plot([q[0]+1, q[0]+1], [q[1], q[1]+1], 'k')

plt.grid()
plt.axis('equal')
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Integer based Bresenham algorithm")
plt.show()



#######################################
#######################################
# Library Implementation of Bresenham #
# Note: you can run this for any (x1, y1, x2, y2)
from bresenham import bresenham
line = (0, 0, 7, 5)

cells = list(bresenham(line[0], line[1], line[2], line[3]))
print(cells)

plt.plot([line[0], line[2]], [line[1], line[3]])


for q in cells:
    plt.plot([q[0], q[0]+1], [q[1], q[1]], 'k')
    plt.plot([q[0], q[0]+1], [q[1]+1, q[1]+1], 'k')
    plt.plot([q[0], q[0]], [q[1],q[1]+1], 'k')
    plt.plot([q[0]+1, q[0]+1], [q[1], q[1]+1], 'k')

plt.grid()
plt.axis('equal')
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Python package Bresenham algorithm")
plt.show()