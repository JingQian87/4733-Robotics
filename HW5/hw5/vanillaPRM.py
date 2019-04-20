"""
	Goal: Build a probabilistic roadmap and visualize it on the environment, along with the shortest path. 
	Steps: 
		0. import map from visualize_map
		1. generate nodes: sample configurations uniformly
		2. use k-nearest-neighbors to find edges.
		3. add start and goal to the graph and find edges to the map
		4. graph search (use functions from hw4) 
	Notice: 
		1. Set the number of samples and number of nearest neighbors as free parameters 
		2. The graph may not be connected and could not find path. 
		3. Due to the randomness of sampling, the shortest path may vary.
"""
import numpy as np
import random, math
import matplotlib.pyplot as plt 
from sklearn.neighbors import NearestNeighbors

from visualize_map import *
from shortest_path import *

obstacle_file = 'world_obstacles.txt'
start_goal_file = 'start_goal.txt'

def generate_nodes(path_data, nsample):
	V = []
	# sample uniformly until have nsample collision-free nodes.
	xy_min = [0, 0]
	xy_max = [600, 600]
	while len(V) < nsample:
		q = np.random.uniform(low=xy_min, high=xy_max, size=(1,2))[0]
		if not path_data.contains_point(q):
			V.append(q)
	return V

def edges_obstacles(path_data):
	E_obstacles = []
	for i in range(1, len(path_data)-1):
		if path_data.codes[i] == 1: 
			start = path_data.vertices[i]
		elif path_data.codes[i] == 79:
			continue
		elif path_data.codes[i+1] == 79:
			E_obstacles.append((path_data.vertices[i], start))
			continue
		E_obstacles.append((path_data.vertices[i], path_data.vertices[i+1]))
	return np.array(E_obstacles)

def ccw(A,B,C):
	return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])

def intersect(A,B,C,D):
	return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def generate_edges(path_data, nodes, k):
	""" Find k nearest neighbors for each node.
		In the sklearn.neighbors.NearestNeighbors, k=1 returns the point itself.
		First column of indices is q, and the rest columns are its neighbors' indices.
		Similarly, the first column of distances is 0, and the rest columns are the distance from q to its k nearest neighbors.
	"""
	nbrs = NearestNeighbors(n_neighbors=k+1, algorithm='brute').fit(nodes)
	distances, indices = nbrs.kneighbors(nodes)
	#print(distances, indices)

	E_obs = edges_obstacles(path_data)
	E = []

	for i in range(len(nodes)):
		q = tuple(nodes[i].tolist())
		kindices = indices[i, 1:]
		#print(q, knbrs)
		for j in kindices:
			qj = tuple(nodes[j].tolist())
			if (q, qj) not in E:# and (q, qj) not in E:
				for l in E_obs:
					if intersect(q, qj, l[0], l[1]):
						break
				else:
					E.append((q, qj))
	return E, E_obs

def find_sg_knear(point, nodes, k):
	dist = dict()
	for i in nodes:
		dist[tuple(i.tolist())] = manhattan_dist(i, point)
	knear = []
	for i in range(k):
		tmp = min(dist, key=dist.get)
		knear.append(tmp)
		del dist[tmp]
	return knear

def add_start_goal(start, goal, nodes, obs_edges, k_start_goal):
	newEdge = []
	knear_start = find_sg_knear(start, nodes, k_start_goal)
	for i in range(k_start_goal):
		qj = knear_start[i]
		for l in obs_edges:
			if intersect(start, qj, l[0], l[1]):
				break
		else:
			newEdge.append((start, qj))
	knear_goal = find_sg_knear(goal, nodes, k_start_goal)
	for i in range(k_start_goal):
		qj = knear_goal[i]
		for l in obs_edges:
			if intersect(goal, qj, l[0], l[1]):
				break
		else:
			newEdge.append((qj, goal))
	return newEdge



def new_spath(start, goal, edges):
	# Compute adjacency list
	adj_list = defaultdict(list)
	for edge in edges:
		edge_len = manhattan_dist(edge[0], edge[1])
		adj_list[edge[0]].append((edge[1], edge_len))
		adj_list[edge[1]].append((edge[0], edge_len))
	# print(adj_list)

	return dijkstra(start, goal, adj_list)


if __name__ == "__main__":
	fig, ax = plt.subplots()
	path = build_obstacle_course(obstacle_file, ax)
	start, goal = add_start_and_goal(start_goal_file, ax)
	V = generate_nodes(path, 500)
	E, E_obs = generate_edges(path, V, 5)
	E_added = add_start_goal(start, goal, V, E_obs, 5)
	for e in E_added:
		E.append(e)
	for e in E:
		ep = Path(e)
		epatch = patches.PathPatch(ep, facecolor='None', edgecolor='xkcd:lightblue')
		ax.add_patch(epatch)

	points, path_len = new_spath(start, goal, E)
	xs = [point[0] for point in points]
	ys = [point[1] for point in points]
	plt.plot(xs, ys, 'k--', lw=1)
	plt.savefig("map_vanillaPRM.pdf")
	#plt.show()

