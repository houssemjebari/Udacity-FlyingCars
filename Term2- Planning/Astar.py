import numpy as np
from queue import PriorityQueue
from enum import Enum

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

def heuristic(position, goal_position):
    h = np.linalg.norm(np.array(goal_position) - np.array(position))
    return h


def a_star(grid,heuristic,start,goal):
    open = PriorityQueue()
    visited = set()
    branch = dict()
    open.put((0+heuristic(start,goal),start))
    visited.add(start)
    found = False

    while(not open.empty()):
        current = open.get()
        current_node = current[1]
        current_cost = current[0]
        if current_node == goal:
            print("Path found")
            found = True
            break
        else:
           actions = valid_actions(grid, current_node) 
           for action in actions :
                d = action.delta  
                next_node = (current_node[0]+d[0],
                            current_node[1]+d[1])
                if next_node not in visited and grid[next_node[0],next_node[1]]==0:
                   G = current_cost + action.cost - heuristic(current_node,goal)
                   H = heuristic(next_node,goal)
                   open.put((G+H,next_node))
                   visited.add(next_node)
                   branch[next_node] = current_node
    if found:
        path = []
        n = goal 
        while (n != start):
            path.append(n)
            n = branch[n]
        return path 
    


start = (0, 0)
goal = (4, 5)

grid = np.array([
    [0, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 0, 1, 1, 0],
    [0, 0, 0, 1, 0, 0],
])

path = a_star(grid, heuristic, start, goal)
print(path)
