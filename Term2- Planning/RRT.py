import copy as cp
import numpy as np 
import matplotlib.pyplot as plt
import networkx as nx
from sklearn.neighbors import KDTree
plt.rcParams['figure.figsize'] = 12,12


ANGLE_STDDEV = np.deg2rad(3)
MAX_STEERING_ANGLE = np.deg2rad(30)

class RRT:
    def __init__(self,x_init):
        self.tree = nx.DiGraph()
        self.tree.add_node(x_init)
    
    def add_vertex(self,x_new):
        self.tree.add_node(tuple(x_new))
    
    def add_edge(self,x_near,x_new,u):
        self.tree.add_edge(tuple(x_near),tuple(x_new),orientation=u)

    @property
    def vertices(self):
        return list(self.tree.nodes())

    @property
    def edges(self):
        return self.tree.edges()


def create_grid():
    grid = np.zeros((100, 100))
    # build some obstacles
    grid[10:20, 10:20] = 1
    grid[63:80, 10:20] = 1
    grid[43:60, 30:40] = 1
    grid[71:86, 38:50] = 1
    grid[10:20, 55:67] = 1
    grid[80:90, 80:90] = 1
    grid[75:90, 80:90] = 1
    grid[30:40, 60:82] = 1
    return grid


def sample_state(grid):
    x = np.random.uniform(0,grid.shape[0])
    y = np.random.uniform(0,grid.shape[1])
    return (x,y)


def nearest_neighbor(x_rand,rrt):
    tree = KDTree(rrt.vertices)
    index = tree.query([x_rand],1,return_distance=False)[0][0]
    return rrt.vertices[index]


def select_input(x_rand,x_near):
    angle = np.arctan2(x_rand[1]-x_near[1],x_rand[0]-x_near[0])
    return angle

def new_state(x_near,u,dt):
    new_x = x_near[0] + np.cos(u)*dt
    new_y = x_near[1] + np.sin(u)*dt
    return [new_x,new_y]

def generate_RRT(grid,x_init,num_vertices,dt):
    rrt = RRT(x_init)
    for _ in range(num_vertices):
        x_rand = sample_state(grid)
        while(grid[int(x_rand[0]),int(x_rand[1])] == 1):
            x_rand = sample_state(grid)
            x_near = nearest_neighbor(x_rand,rrt)
            u = select_input(x_rand,x_near)
            x_new = new_state(x_near,u,dt) 
            if x_new[0]<grid.shape[0] and x_new[1]<grid.shape[1]:
                if grid[int(x_new[0]),int(x_new[1])] == 0:
                    rrt.add_edge(x_near,x_new,u)        
    return rrt


num_vertices = 2000
dt = 2
x_init = (50, 50)

grid = create_grid()
rrt = generate_RRT(grid, x_init, num_vertices, dt)
plt.imshow(grid, cmap='Greys', origin='lower')
plt.plot(x_init[1], x_init[0], 'ro')

for (v1, v2) in rrt.edges:
    plt.plot([v1[1], v2[1]], [v1[0], v2[0]], 'y-')

plt.show()