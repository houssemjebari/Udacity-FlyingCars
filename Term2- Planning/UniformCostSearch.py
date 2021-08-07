import numpy as np 
from enum import Enum 
from queue import PriorityQueue

class Action(Enum):
    """ 
    An action is represented by a 3 element
    tuple.
    The first 2 values are the delta of the 
    action relative to the current grid pos-
    ition.
    """
    LEFT = (0,-1,1)
    RIGHT = (0,1,1)
    UP = (-1,0,1)
    DOWN = (1,0,1)

    def __str__(self):
        if self == self.LEFT:
            return '<'
        if self == self.RIGHT:
            return '>'
        if self == self.UP:
            return '^'
        if self == self.DOWN:
            return '-'
    @property
    def cost(self):
        return self.value[2]
    
    @property
    def delta(self):
        return (self.value[0],self.value[1])
    
def valid_actions(grid,current_node):
    """
    Returns a list of valid actions
    given a grid and current node.
    """
    valid = [Action.UP, Action.LEFT,
            Action.RIGHT,Action.DOWN]
    n,m = grid.shape[0]-1,grid.shape[1]-1
    x,y = current_node

    if x-1<0 or grid[x-1,y]==1:
        valid.remove(Action.UP)
    if x+1>n or grid[x+1,y]==1:
        valid.remove(Action.DOWN)
    if y-1<0 or grid[x,y-1]==1:
        valid.remove(Action.LEFT)
    if y+1>m or grid[x,y+1]==1:
        valid.remove(Action.RIGHT)
    return valid


def uniform_cost(grid,start,goal):
    path = []
    queue = PriorityQueue()
    visited = set()
    queue.put((0,start))
    visited.add(start)
    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]
        if current_node == goal:
            print('Found a path')
            found = True
            break
        else:
            for action in valid_actions(grid,current_node):
                delta = action.delta
                cost = action.cost
                next_node = (current_node[0] + delta[0], current_node[1] + delta[1])
                new_cost = (current_cost + cost)
                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost,next_node))
                    branch[next_node] = current_node
    if found:
        n = goal
        while(n!= start):
            path.append(n)
            n = branch[n]
        path.append(n)
        return path


start = (0, 0)
goal = (4, 4)

grid = np.array([
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, 1, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, 0, 1, 0],
    [0, 0, 0, 1, 0, 0],])


path = uniform_cost(grid, start, goal)
print(path)

