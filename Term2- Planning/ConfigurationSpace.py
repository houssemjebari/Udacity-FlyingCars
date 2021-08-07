import numpy as np 
import matplotlib.pyplot as plt 

# Initialize figure parameter
plt.rcParams["figure.figsize"]=[12,12]

# Read the data
filename = 'colliders.csv'
data = np.loadtxt(filename,delimiter=',',dtype=np.float64,skiprows=2)

# Define Parameters
drone_altitude = 5
safe_distance = 0.4


def create_grid(data,drone_altitude,safety_distance):
    """ 
    Returns a grid representation of a 2D configuration
    space based on given obstacle data, drone altitude 
    and safety distance"""
    min_north = np.floor(np.amin(data[:,0])-np.amin(data[:,3]))
    max_north = np.ceil(np.amax(data[:,0])+np.amax(data[:,3]))
    min_east = np.floor(np.amin(data[:,1])-np.amin(data[:,4]))
    max_east = np.ceil(np.amax(data[:,1])-np.amin(data[:,4]))

    north_size = int(max_north-min_north)
    east_size = int(max_east-min_east)
    grid = np.zeros((north_size,east_size))

    for d in data:
        if d[2] + d[5] > drone_altitude:
            obstacles = [int(np.clip(d[0] - d[3] - safety_distance - min_north,0,north_size-1)),
                         int(np.clip(d[0] + d[3] + safety_distance - min_north,0,north_size-1)),
                         int(np.clip(d[1] - d[4] - safety_distance - min_east,0,east_size-1)),
                         int(np.clip(d[1] + d[4] + safety_distance - min_east,0,east_size-1 ))
                        ]
            grid[obstacles[0]:obstacles[1],obstacles[2]:obstacles[3]] = 1
    return grid


grid = create_grid(data, drone_altitude, safe_distance)
# equivalent to
# plt.imshow(np.flip(grid, 0))
# NOTE: we're placing the origin in the lower lefthand corner here
# so that north is up, if you didn't do this north would be positive down
plt.imshow(grid, origin='lower',cmap="gray") 

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()
