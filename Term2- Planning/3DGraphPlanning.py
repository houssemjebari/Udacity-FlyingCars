"""
This File contains helper functions and visualizations of Probabilistic Road Map (PRM)
The implemented functions can be used for global planning with Unmanned Aerial Vehicles (UAVs)
since it plans paths through 3D environment given static obstacles data
"""


import time 
import os
from networkx.generators.geometric import euclidean
import numpy as np 
import networkx as nx
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from shapely.geometry import Polygon, Point,LineString
from sklearn.neighbors import KDTree
from queue import PriorityQueue


# Define Parameters
SAFETY_DISTANCE = 3
DRONE_ALTITUDE = 7
VOXEL_SIZE = 10


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
    north_size = int(max_north - min_north) 
    # Get the east size
    min_east = np.floor(np.min(data[:,1] - data[:,4]))
    max_east = np.ceil(np.max(data[:,1] + data[:,4]))
    east_size = int(max_east - min_east) 
    # Build the array 
    grid = np.zeros((north_size,east_size))
    for i in range(data.shape[0]):
        if (DRONE_ALTITUDE < (data[i,2] + data[i,5])):
            min_north_obstacle = int(np.clip(np.floor(data[i,0] - data[i,3] - SAFETY_DISTANCE - min_north),0,north_size-1)) 
            max_north_obstacle = int(np.clip(np.ceil(data[i,0] + data[i,3] + SAFETY_DISTANCE - min_north),0,north_size-1))  
            min_east_obstacle = int(np.clip(np.floor(data[i,1] - data[i,4] - SAFETY_DISTANCE - min_east),0,east_size-1)) 
            max_east_obstacle = int(np.clip(np.floor(data[i,1] + data[i,4] + SAFETY_DISTANCE - min_east),0,east_size-1)) 
            grid[min_north_obstacle:max_north_obstacle,min_east_obstacle:max_east_obstacle] = 1 
    return grid ,min_north,min_east


def create_vox_map(data):
    """
    Computes a 3D map of the environmenet 
    from colliders data
    
    Args:
        data - numpy array of shape (n,5) containing 
               collision data where n is the number
               of polygons representing obstaclees
    """
    # Get the north size
    min_north = np.floor(np.min(data[:,0] - data[:,3]))
    max_north = np.ceil(np.max(data[:,0] + data[:,3]))
    north_size = int(max_north - min_north) // VOXEL_SIZE
    # Get the east size
    min_east = np.floor(np.min(data[:,1] - data[:,4]))
    max_east = np.ceil(np.min(data[:,1] + data[:,4]))
    east_size = int(max_east - min_east) // VOXEL_SIZE
    # Get the altitude 
    altitude = np.ceil(np.max(data[:,2]+data[:,5]))
    altitude_size = int(altitude) // VOXEL_SIZE
    # Build the array 
    grid = np.zeros((north_size,east_size,altitude_size))
    for i in range(data.shape[0]):
        min_north_obstacle = int(np.floor(data[i,0] - data[i,3] - north_size)) // VOXEL_SIZE
        max_north_obstacle = int(np.ceil(data[i,0] + data[i,3] - north_size)) // VOXEL_SIZE
        min_east_obstacle = int(np.floor(data[i,1] - data[i,4] - east_size)) // VOXEL_SIZE
        max_east_obstacle = int(np.ceil(data[i,1] + data[i,4] - north_size)) // VOXEL_SIZE
        altitude_obstacle = int(np.floor(data[i,2] + data[i,5])) // VOXEL_SIZE
        grid[min_north_obstacle:max_north_obstacle,min_east_obstacle:max_east_obstacle,0:altitude_obstacle] = 1
    return grid


def get_polygons(data):
    """
    Gets polygons from obstacles data 
    Args:
        data - numpy array of shape (n,5) containing 
               collision data where n is the number
               of polygons representing obstaclees
    """
    polygons = []
    for i in range(data.shape[0]):
        min_north = data[i,0] - data[i,3]
        max_north = data[i,0] + data[i,3]
        min_east = data[i,1] - data[i,4]
        max_east = data[i,1] + data[i,4]
        altitude = data[i,2] + data[i,5]
        polygon = Polygon([[min_north,max_east],
                           [min_north,min_east],
                           [max_north,min_east],
                           [max_north,max_east]])
        polygons.append((polygon,altitude))
    return polygons


def sample_points(data,polygons,size):
    """
    Samples random points in the environment, 
    checks for collision with obstacles and 
    returns a set of collision-free points
    
    Args:
        data - numpy array of shape (n,5) containing 
               collision data where n is the number
               of polygons representing obstaclees
        polygons - list containing all polygons in colliders 
                    data
        size - int, of the number of points to sample
    """

    collision_free_samples = []
    # Get samples on the form (x,y,z) tuples
    xmin = np.min(data[:,0] - data[:,3])
    xmax = np.max(data[:,0] + data[:,3])
    ymin = np.min(data[:,1] - data[:,4])
    ymax = np.max(data[:,1] + data[:,4])
    altitude = np.max(data[:,2]+data[:,5])
    x = np.random.uniform(low=xmin,high=xmax,size=size)
    y = np.random.uniform(low=ymin,high=ymax,size=size)
    z = np.random.uniform(low=0,high=altitude,size=size)
    samples = list(zip(x,y,z))
    # Get the polygons and instantiate a KDTree
    centers = [p[0].centroid.coords[0] for p in polygons]
    tree = KDTree(centers,metric='euclidean')
    # Search of collision free samples
    for sample in samples:
        max_poly_xy = 2 * np.max((data[:, 3], data[:, 4]))
        indices = list(tree.query_radius(np.array([sample[0], sample[1]]).reshape(1, -1), r=max_poly_xy)[0])
        collision = False
        for index in indices:
            if not polygons[index][0].crosses(Point(sample)) or polygons[index][1]<sample[2]:
                 collision = True
                 break
        if not collision:
            collision_free_samples.append(sample)
    return collision_free_samples


def construct_graph(nodes,polygons,k_neighbors=5):
    """
    Constructs a graph from given nodes
    and collision data

    Args:
        nodes - list of (x,y,z) tuples
        data - numpy array of shape (n,5) containing 
               collision data where n is the number
               of polygons representing obstaclees
        k_neighbors - int, representing the number of 
                    neighbor nodes to be connected
    """
    # Create a graph 
    graph = nx.Graph()
    # Create a tree of nodes to search it
    print(len(nodes))
    tree = KDTree(nodes)
    # Search for nearest neighbors for each node
    # and check if the connection intersects with
    # any polygon
    for node in nodes:
        indices = list(tree.query([node],k_neighbors,return_distance=False)[0])
        for index in indices:
            node2 = nodes[index]
            if node == node2:
                continue
            else:
                line = LineString([node,node2])
                infeasible = False
                for polygon in polygons:
                    if polygon[0].crosses(line) and polygon[1] >= min(node[2],node2[2]):
                        infeasible = True
                        break
                if not infeasible:
                    graph.add_edge(node,node2)
    return graph
    

def heuristic(point1,point2):
    """
    Computes a euclidean distance of two given 
    points in space

    args:
        point1 - a tuple of (x,y) position
        point2 - a tuple of (x,y) position 
    """
    distance = np.linalg.norm(np.array([point1[0]-point2[0],point1[1]-point2[1],point1[2]-point2[2]]))
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
        path.append(start)
    else:
        print("Path not found !")

    return path



def test_voxmap():
    filename = "colliders.csv"
    data = read_file(filename)
    voxmap = create_vox_map(data)
    # Plot the voxel map 
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.voxels(voxmap,edgecolors='k')
    plt.show()


def main():
    # Construct graph from colliders
    pipeline_start = time.time()
    filename = "colliders.csv"
    data = read_file(filename)
    polygons = get_polygons(data)
    nodes = sample_points(data,polygons,500)
    graph = construct_graph(nodes,polygons,5)
    graph_building_time = time.time() - pipeline_start

    # Perform Path planning 
    search_start = time.time()
    start_ne = (25,  100.,5.)
    goal_ne = (720., 370.,7.) # changed from (720,370)
    start = closest_point(graph,start_ne)
    goal = closest_point(graph,goal_ne)
    print("Closest starting point: ",start)
    print("Closest goal point: ",goal)
    path = a_star(graph,heuristic,start,goal)
    search_time = time.time() - search_start
    pipeline_time = time.time() - pipeline_start
    """# Plot the voxel map 
    voxmap = create_vox_map(data)
    fig1 = plt.figure()
    ax = fig1.gca(projection='3d')
    ax.voxels(voxmap,edgecolor='k',color='b')
    plt.show()"""
    
    # Plot the planned data
    grid,north_offset,east_offset = create_grid(data)
    plt.imshow(grid,origin='lower',cmap = "Greys")
    for n1 in graph.nodes:
        plt.scatter(n1[1] - east_offset, n1[0] - north_offset, c='red')
    
    # draw edges
    for (n1, n2) in graph.edges:
        plt.plot([n1[1] - east_offset, n2[1] - east_offset], [n1[0] - north_offset, n2[0] - north_offset], 'black')
    
    # TODO: add code to visualize the path
    path_pairs = zip(path[:-1], path[1:])
    for (n1, n2) in path_pairs:
        plt.plot([n1[1] - east_offset, n2[1] - east_offset], [n1[0] - north_offset, n2[0] - north_offset], 'green')


    plt.xlabel('NORTH')
    plt.ylabel('EAST')

    plt.show()
    print ('Total Pipeline time: ',pipeline_time) # 15.375 s
    print ('Total graph constructing time: ',graph_building_time) # 15.372 s
    print('search time: ',search_time) # 0.002 s

def test():
    p = Polygon([[1,0],[1,1],[0,1],[0,0]])
    print(list(p.centroid.coords[0]))
    data = np.array([[1,2,3,4,5],[6,7,8,9,10]])
    print(2 * np.max((data[:, 3], data[:, 4]),axis=1))

if __name__ == "__main__":
    main()
        






        