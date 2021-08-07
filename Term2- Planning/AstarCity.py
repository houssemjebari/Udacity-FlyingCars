"""
In this notebook you'll combine the work of previous exercises to calculate a minimal series of waypoints in order to get from a start location to a goal location.

You'll reuse and modify your algorithms from:

- A*
- Configuration Space
- Collinearity and/or Bresenham
"""

# Import all the necessary libraries
from tokenize import Hexnumber
import numpy as np 
import matplotlib.pyplot as plt 
import argparse
import time  
import os
import copy
from enum import Enum, auto
from queue import PriorityQueue




# Define global parameters
DRONE_ALTITUDE = 3
SAFE_DISTANCE = 3
RESOLUTION = 1

# Define Actions 
class Action(Enum):
    """
    An action is represented by a 3 element tuple.
    
    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """
    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)
    UP_RIGHT = (-1, 1, np.sqrt(2))
    UP_LEFT = (-1, -1, np.sqrt(2))
    DOWN_LEFT = (1, -1, np.sqrt(2))
    DOWN_RIGHT = (1, 1, np.sqrt(2))
    @property
    def cost(self):
        return self.value[2]
    
    @property
    def delta(self):
        return (self.value[0], self.value[1])



# Define Helper functions
def read_data(filename):
    if os.path.isfile(filename):
        data = np.loadtxt(filename,delimiter=',',dtype='Float64',skiprows=2)
        return np.array(data)

def create_grid(collision_data):
    min_north = np.floor(np.min(collision_data[:,0] - collision_data[:,3]))
    max_north = np.ceil(np.max(collision_data[:,0] + collision_data[:,3]))
    min_east = np.floor(np.min(collision_data[:,1] - collision_data[:,4]))
    max_east = np.ceil(np.max(collision_data[:,1] + collision_data[:,4]))
    north_size = int((RESOLUTION**-1) * max_north-min_north)
    east_size = int((RESOLUTION**-1) * max_east-min_east)
    grid = np.zeros((north_size,east_size))
    for i in range(collision_data.shape[0]):
        if collision_data[i][2] > DRONE_ALTITUDE:
            min_east_collision =int(np.clip(np.floor(collision_data[i][0]-collision_data[i][3]-SAFE_DISTANCE-min_north),0,north_size-1))
            max_east_collision = int(np.clip(np.ceil(collision_data[i][0]+collision_data[i][3]-min_north + SAFE_DISTANCE),0,north_size-1))
            min_north_collision = int(np.clip(np.floor(collision_data[i][1]-collision_data[i][4]-SAFE_DISTANCE-min_east),0,east_size-1))
            max_north_collision = int(np.clip(np.ceil(collision_data[i][1]+collision_data[i][4]-min_east + SAFE_DISTANCE),0,east_size-1))
            grid[min_east_collision:max_east_collision,
                 min_north_collision:max_north_collision] = 1
    return grid


def heuristic(position, goal_position):
    h = np.linalg.norm(np.array(goal_position) - np.array(position))
    return h


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = [Action.UP, Action.LEFT, Action.RIGHT, Action.DOWN, Action.UP_RIGHT, Action.UP_LEFT, Action.DOWN_LEFT, Action.DOWN_RIGHT]
    n, m = grid.shape[0] -1 , grid.shape[1]-1
    x, y = current_node[0],current_node[1]
    
    # check if the node is off the grid or
    # it's an obstacle
    
    if x - 1 < 0 :
        valid.remove(Action.UP)
    
    if x + 1 > n:
        valid.remove(Action.DOWN)
    
    if y - 1 < 0:
        valid.remove(Action.LEFT)
    
    if y + 1 > m :
        valid.remove(Action.RIGHT)
    
    if ((x-1) < 0 or (y+1) > m):
        valid.remove(Action.UP_RIGHT)
    
    if (x-1) < 0 or (y-1) < 0:
        valid.remove(Action.UP_LEFT)
    
    if (x+1) > n or (y-1) < 0:
        valid.remove(Action.DOWN_LEFT)
    
    if (x+1) > n or (y+1) > m:
        valid.remove(Action.DOWN_RIGHT)
    return valid


def Astar(grid,start,goal):
    open = PriorityQueue()
    visited = set()
    branch = dict()
    open.put((heuristic(start,goal),start))
    visited.add(start)
    found = False

    while not open.empty():
        current = open.get()
        current_node = current[1]
        current_cost = current[0]
        if current_node == goal:
            found = True
            break
        else:
            actions = valid_actions(grid,current_node)
            for action in actions:
                    d = action.delta  
                    new_node = (current_node[0]+d[0],current_node[1]+d[1])
                    new_node_cost = current_cost + action.cost - heuristic(current_node,goal) + heuristic(new_node,goal)
                    if new_node not in visited:
                        if grid[new_node[0],new_node[1]]==0:
                            open.put((new_node_cost,new_node))
                            visited.add(new_node)
                            branch[new_node] = current_node

    if found:
        print('Path Found')
        path = []
        n = goal 
        while n!=start:
            path.append(n)
            n = branch[n]
        return path 
    else:
        print("Impossible to find a path")     

def point(p):
    """
    Recieves a tuple of (x,y) point and returns a 3d vector
    """
    return np.array([p[0],p[1],1.0]).reshape(1,3)

def colinearity_check(p1,p2,p3,epsilon=1e-6):
    """
    Checks if three points are colinear
    """
    vector = np.vstack((point(p1),point(p2),point(p3)))
    return abs(np.linalg.det(vector)) < epsilon

def bresenham(p1,p2):
    """
    Returns all the grid cells that are included 
    in the line between the two given points 
    """
    x1,y1 = p1
    x2,y2 = p2
    m = (y2 - y1)/(x2 - x1)
    cells = []
    while x1<x2:
        cells.append((x1,y1))
        if (x1+1)*m < y1+1:
            x1 += 1
        elif (x1+1)*m < y1+1:
            x1 += 1
            y1 += 1
        else:
            y1 += 1
    return cells


def prune_path(path):
    """
    We are here using the colinearity check
    but we can also use bresenham
    """
    pruned_path = copy.deepcopy(path)
    i = 0
    while i < len(pruned_path) - 2:
        p1 = pruned_path[i]
        p2 = pruned_path[i+1]
        p3 = pruned_path[i+2]
        if colinearity_check(p1,p2,p3):
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path

def test_Astar():
    start = (0,0)
    goal = (4, 5)

    grid = np.array([[0, 1, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0],
                     [0, 0, 0, 1, 1, 0],
                     [0, 0, 0, 1, 0, 0],])

    path = Astar(grid, start, goal)
    print(path)

def test_grid():
    filename = 'colliders.csv'
    data = read_data(filename)
    grid = create_grid(data)
    plt.imshow(grid,cmap="gray",origin='lower')
    plt.show()

def main():
    pipeline_start = time.time()
    filename = 'colliders.csv'
    data = read_data(filename)
    grid = create_grid(data)
    start = (25,  175)
    goal = (750., 370.)
    search_start = time.time()
    path = Astar(grid,start,goal)
    search_time = time.time() - search_start
    pruned_path = prune_path(path)
    pipeline_time = time.time() - pipeline_start
    # Print the path
    plt.imshow(grid, cmap='gray',origin='lower')
    x_val_prune = [x[0] for x in pruned_path[::-1]]
    y_val_prune = [y[1] for y in pruned_path[::-1]]
    x_val = [x[0] for x in path[::-1]]
    y_val = [y[1] for y in path[::-1]]
    plt.plot(y_val_prune,x_val_prune, color = 'blue')
    plt.scatter(y_val,x_val)
    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    plt.show()
    # Pipeline Performance
    print('Constructing model and planning took ',pipeline_time,' seconds') # 26.328 s
    print('Search on 2D graph took ',search_time,' seconds')# 25.77 s

if __name__ == "__main__":
    main()




