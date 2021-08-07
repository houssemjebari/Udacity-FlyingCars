""" In this exercice we'll implement
breadth-first search to find a path 
from start to goal in a grid world
"""

from threading import current_thread
import numpy as np 
from queue import Queue
from enum import Enum

class Action(Enum):
    LEFT = (0,-1)
    RIGHT = (0,-1)
    UP = (-1,0)
    DOWN = (1,0)

    def __str__(self):
        if self == self.LEFT:
            return '<'
        elif self == self.RIGHT:
            return '>'
        elif self == self.UP:
            return '^'
        elif self == self.DOWN:
            return 'v'


def breadth_first_search(grid,start,goal):
    # Define open, visited and partial paths    
    open = Queue()
    visited = set()
    branch = dict()

    # Initialize open and visited lists 
    open.put(start)
    visited.add(start)

    # Search for the path
    current_node = start 
    while (not open.empty()):
        if (current_node==goal):
            path = []
            n = goal
            print("path found")
            while n!=start:
                path.append(n)
                n = branch[n]
            path.append(n)
            return path
        else:
            current_node = open.get()
            if current_node[0] != 0:
                if ((grid[current_node[0]-1][current_node[1]] != 1) and (current_node[0]-1,current_node[1]) not in visited):
                    next_node = (current_node[0]-1,current_node[1]) 
                    open.put(next_node)
                    visited.add(next_node)
                    branch[next_node] = current_node

            if current_node[0] != (grid.shape[0]-1):
                if ((grid[current_node[0]+1][current_node[1]] != 1) and  (current_node[0]+1,current_node[1]) not in visited):
                    next_node = (current_node[0]+1,current_node[1]) 
                    open.put(next_node)
                    visited.add(next_node)
                    branch[next_node] = current_node

            if current_node[1] != 0:
                if ((grid[current_node[0]][current_node[1]- 1] != 1) and  (current_node[0],current_node[1]-1) not in visited):
                    next_node = (current_node[0],current_node[1]-1) 
                    open.put(next_node)
                    visited.add(next_node)
                    branch[next_node] = current_node
            if current_node[0] != (grid.shape[1]-1):
                if ((grid[current_node[0]][current_node[1]+1] != 1) and (current_node[0],current_node[1]+1) not in visited):
                    next_node = (current_node[0],current_node[1]+1) 
                    open.put(next_node)
                    visited.add(next_node)
                    branch[next_node] = current_node


grid = np.array([
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, 1, 0, 0],
    [0, 0, 0, 1, 1, 0],
    [0, 0, 0, 1, 0, 0]])

# Define start and goal location
start = (1,0)
goal = (1,4)

path = breadth_first_search(grid,start,goal)
print(path)


