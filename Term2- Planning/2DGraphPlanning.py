import time
import os
import numpy as np 
import matplotlib.pyplot as plt
import networkx as nx
from scipy.spatial import Voronoi
from queue import PriorityQueue
from matplotlib import colors
from bresenham import bresenham


# Define Parameters
SAFETY_DISTANCE = 3
DRONE_ALTITUDE = 5


def read_file(filename):
    """
    Reads the file data from given filename
    
    Args:
        filename - name of the file to be read
    """
    if os.path.isfile(filename):
        data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
        return data


def create_grid(data):
    """
    Creates a configuration state from the
    collision data provided

    Args:
        data - numpy array of shape (n,5) containing 
               collision data where n is the number
               of polygons representing obstaclees
    """
    # Get the north size
    min_north = np.floor(np.min(data[:,0] - data[:,3]))
    max_north = np.ceil(np.max(data[:,0] + data[:,3]))
    north_size = int(np.ceil(max_north - min_north))
    # Get the east size
    min_east = np.floor(np.min(data[:,1] - data[:,4]))
    max_east = np.ceil(np.max(data[:,1] + data[:,4]))
    east_size = int(np.ceil(max_east - min_east))
    # Build the array 
    points = []
    grid = np.zeros((north_size,east_size))
    for i in range(data.shape[0]):
        points.append([data[i,0]-min_north,data[i,1]-min_east])
        if (DRONE_ALTITUDE < (data[i,2] + data[i,5] + SAFETY_DISTANCE)):
            min_north_obstacle = int(np.clip(np.floor(data[i,0] - data[i,3] - SAFETY_DISTANCE - min_north),0,north_size-1))
            max_north_obstacle = int(np.clip(np.ceil(data[i,0] + data[i,3] + SAFETY_DISTANCE - min_north),0,north_size-1))  
            min_east_obstacle = int(np.clip(np.floor(data[i,1] - data[i,4] - SAFETY_DISTANCE - min_east),0,east_size-1))
            max_east_obstacle = int(np.clip(np.ceil(data[i,1] + data[i,4] + SAFETY_DISTANCE - min_east),0,east_size-1))
            grid[min_north_obstacle:max_north_obstacle+1,min_east_obstacle:max_east_obstacle+1] = 1 
    return grid,points 


def vonoroi_graph(grid,points):
    graph = Voronoi(points)
    # TODO: check each edge from graph.ridge_vertices for collision
    edges = []
    for edge in graph.ridge_vertices:
        point1 = graph.vertices[edge[0]]
        point2 = graph.vertices[edge[1]]
        
        cells = list(bresenham(int(point1[0]), int(point1[1]), int(point2[0]), int(point2[1])))
        infeasible = False
        
        for cell in cells:
            if np.amin(cell) < 0 or cell[0] >= grid.shape[0] or cell[1] >= grid.shape[1]:
                infeasible = True
                break
            if grid[cell[0], cell[1]] == 1:
                infeasible = True
                break
        if infeasible == False and len(cells)>=1:
            point1 = (point1[0], point1[1])
            point2 = (point2[0], point2[1])
            edges.append((point1,point2))

    return edges


def construct_graph(edges):
    """
    Creates a graph given a list of edge points
    """
    graph = nx.Graph()
    for edge in edges:
        graph.add_edge(edge[0],edge[1])
    return graph


def heuristic(point1,point2):
    """
    Computes a euclidean distance of two given 
    points in space

    args:
        point1 - a tuple of (x,y) position
        point2 - a tuple of (x,y) position 
    """
    distance = np.sqrt((point1[0]-point2[0])**2 +(point1[1]-point2[1])**2)
    return distance


def closest_point(graph,point):
    """
    Computes the closest point in the graph 
    from the given point

    args:
        graph - a networkx.Graph object 
        point - a tuple of (x,y) position
    """
    nodes = list(graph.nodes)
    distances = np.linalg.norm(np.array(point)-np.array(nodes),axis=1)
    return nodes[np.argmin(distances)]


def a_star(graph,heuristic,start,goal):
    """
    searchs for a path from a given graph, 
    a starting position and a goal position

    args:
        graph - a networkx.Graph object 
        heuristic - a heuristic function
        start - a tuple of (x,y) position 
        goal - a tuple of (x,y) position 
    """
    open = PriorityQueue()
    visited = set()
    branch = dict()
    open.put((heuristic(start,goal),start))
    visited.add(start)
    found = False
    path = []
    
    while(not open.empty()):
        current = open.get()
        current_node = current[1]
        current_cost = current[0]
        if current_node == goal:
            found = True
            break
        else:
           for node in graph[current_node]:  
               if node not in visited:
                   node_cost = current_cost + heuristic(node,goal) - heuristic(current_node,goal) + heuristic(current_node,node)
                   open.put((node_cost,node))
                   visited.add(node)
                   branch[node] = current_node
    
    if found:
        print("Path found")
        n = goal 
        while (n!=start):
            path.append(n)
            n = branch[n]
    else:
        print("Path not found !")

    return path

def test_bresenham():
    p1 = (0, 0)
    p2 = (7, 5)
    cells = bresenham(p1, p2)
    print(cells)
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]])
    plt.grid()  
    for q in cells:
        plt.plot([q[0], q[0]+1], [q[1], q[1]], 'k')
        plt.plot([q[0], q[0]+1], [q[1]+1, q[1]+1], 'k')
        plt.plot([q[0], q[0]], [q[1],q[1]+1], 'k')
        plt.plot([q[0]+1, q[0]+1], [q[1], q[1]+1], 'k')
        plt.axis('equal')
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Integer based Bresenham algorithm")
    plt.show()

def test_voronoi():
    # Read in the obstacle data
    filename = 'colliders.csv'
    data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
    # Define a flying altitude (feel free to change this)
    grid,points = create_grid(data)
    edges = vonoroi_graph(grid,points)
    print (f'found {len(edges)} edges')
    print('Found %5d edges' % len(edges))
    plt.imshow(grid, origin='lower', cmap='Greys') 
    # Stepping through each edge
    for e in edges:
        p1 = e[0]
        p2 = e[1]
        plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')
    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    plt.show()



def main():
    # Construct graph from colliders
    start_pipeline = time.time()
    filename = "colliders.csv"
    data = read_file(filename)
    grid,points = create_grid(data)
    start_graph = time.time()
    edges = vonoroi_graph(grid,points)
    print (f'found {len(edges)} edges')
    graph = construct_graph(edges)
    graph_time = time.time() - start_graph
    # Perform Path planning 
    start_ne = (25,  100)
    goal_ne = (750., 370.)
    start = closest_point(graph,start_ne)
    print('start location: ', start)
    goal = closest_point(graph,goal_ne)
    print('goal location:', goal)
    start_search = time.time()
    path = a_star(graph,heuristic,start,goal)
    pipeline_time = time.time() - start_pipeline
    search_time = time.time() - start_search
    # Plot results 
    plt.imshow(grid,origin='lower',cmap = "Greys")
    for edge in edges: 
        point1 = edge[0]
        point2 = edge[1]
        plt.plot([point1[1],point2[1]],[point1[0],point2[0]],'b-')
    x_vals = [x[0] for x in path[::-1]]
    y_vals = [y[1] for y in path[::-1]]
    plt.plot(y_vals,x_vals,color='red')
    plt.xlabel("NORTH")
    plt.ylabel('EAST')
    plt.show()
    # Pipeline Performance
    print('Constructing model and planning took ',pipeline_time,' seconds') # 23.8 s
    print('Search on 2D graph took ',search_time,' seconds')# 20 ms
    print('Graph Construction took ',graph_time,' seconds')# 23.04 s


def test():
    arr = np.array([(2,2),(1,1)])
    arr2 = np.array((2,2))
    dist = np.linalg.norm(arr2-arr,axis=1)
    print(dist)
    print(np.argmin(dist))
    print(type(np.sqrt(5**2 - 3**2)))


if __name__ == "__main__":
    main()