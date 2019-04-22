"""
    Performs PRM Enhancement
    
    Steps:
    1. Assign weights to nodes as we are generating edges
        a. num = number of neighbors within distance d
        b. weight = 1 / (num ** 2)
    2. Randomly pick m nodes to enhance based on weight
    3. Generate n more nodes within r distance of each node
        a. Add nodes to V
        b. Add edges to k nearest neighbors to E
"""

from collections import defaultdict
from random import choices
import matplotlib.pyplot as plt
import numpy as np
from sklearn.neighbors import NearestNeighbors

from shortest_path import *
from vanillaPRM import *
from visualize_map import *

obstacle_file = 'world_obstacles.txt'
start_goal_file = 'start_goal.txt'

INI = 500 # initial number of nodes generated
K = 5 # param for knn
D = 30 # number of neighbors within this distance
M = 30 # number of nodes to pick to enhance
N = 5 # number of nodes to generate around each node during enhancement
R = 40 # size of region when generating around nodes

def generate_nodes_enhanced(path_data, nsample, xy_min, xy_max):
    V = []
    while len(V) < nsample:
        q = np.random.uniform(low=xy_min, high=xy_max, size=(1,2))[0]
        if not path_data.contains_point(q):
            V.append(q)
    return V

def generate_edges_enhanced(obstacles, nodes, k, d):
    """ Returns list of edges and map of node weights """
    nbrs = NearestNeighbors(n_neighbors=k+1, algorithm='brute').fit(nodes)
    distances, indices = nbrs.kneighbors(nodes)
    E, weights = [], [0] * len(nodes)

    for i in range(len(nodes)):
        q = tuple(nodes[i].tolist())
        kindices = indices[i, 1:]
        for j in kindices:
            qj = tuple(nodes[j].tolist())
            if (q, qj) not in E and (qj, q) not in E:
                for l in obstacles:
                    if intersect(q, qj, l[0], l[1]):
                        break
                else:
                    E.append((q, qj))
                    if manhattan_dist(q, qj) <= d:
                        weights[i] += 1
                        weights[j] += 1
    
    print(weights)

    for i in range(len(weights)):
        if weights[i] == 0:
            weights[i] = 100
        else:
            weights[i] = 100. / float(weights[i] ** 2)
    return E, weights

def add_edges_for_node(new_node, nodes, obs_edges, k):
    """ Connect a new node to k nearest neighbors in the graph """
    new_node = tuple(new_node.tolist())
    newEdge = []
    knear = find_sg_knear(new_node, nodes, k)
    for i in range(k):
        qj = knear[i]
        for l in obs_edges:
            if intersect(new_node, qj, l[0], l[1]):
                break
        else:
            newEdge.append((new_node, qj))
    return newEdge

if __name__ == "__main__":
    fig, ax = plt.subplots()
    path = build_obstacle_course(obstacle_file, ax)
    start, goal = add_start_and_goal(start_goal_file, ax)
    V = generate_nodes(path, INI) # List of nodes
    E_obs = edges_obstacles(path) # Array of obstacle edges
    E, weights = generate_edges_enhanced(E_obs, V, K, D)
    # print(weights)
    E_added = add_start_goal(start, goal, V, E_obs, 5)
    E += E_added
    for e in E:
        ep = Path(e)
        epatch = patches.PathPatch(ep, facecolor='None', edgecolor='xkcd:lightblue')
        ax.add_patch(epatch)
    
    points, path_len = new_spath(start, goal, E)
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    plt.plot(xs, ys, 'k--', lw=1)
    print("Path length before enhancement: {}".format(path_len))

    ### Randomly pick m nodes to enhance based on weight
    nodes = choices(population=V, weights=weights, k=M)
    # print(nodes)

    ### Generate n more nodes within r distance of each node
    for node in nodes:
        # Generate n more nodes
        xy_min = [min(0, node[0] - R), min(0, node[1] - R)]
        xy_max = [max(600, node[0] + R), max(600, node[1] + R)]
        V_added = generate_nodes_enhanced(path, N, xy_min, xy_max)

        # Add nodes to V
        V += V_added

        # Add edges to k nearest neighbors to E
        for neighbor in V_added:
            E_added = add_edges_for_node(neighbor, V, E_obs, K)
            E += E_added
            for e in E_added:
                ep = Path(e)
                epatch = patches.PathPatch(ep, facecolor='None', edgecolor='xkcd:lightgreen')
                ax.add_patch(epatch)
    
    E_added = add_start_goal(start, goal, V, E_obs, 5)
    E += E_added
    for e in E_added:
        ep = Path(e)
        epatch = patches.PathPatch(ep, facecolor='None', edgecolor='xkcd:lightgreen')
        ax.add_patch(epatch)
    
    points, path_len = new_spath(start, goal, E)
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    plt.plot(xs, ys, 'b--', lw=1)
    print("Path length after enhancement: {}".format(path_len))
    
    plt.show()
