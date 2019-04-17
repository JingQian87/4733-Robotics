"""
	Goal: Build a probabilistic roadmap and visualize it on the environment, along with the shortest path. 
	Steps: 
		0. import map from visualize_map
		1. build graph: sample configurations uniformly and use k-nearest-neighbors to find edges.
		2. graph search (use functions from hw4) 
		*. plot
	Notice: Set the number of samples and number of nearest neighbors as free parameters 
"""
import numpy as np
import random, math
import matplotlib.pyplot as plt 

from visualize_map import *

obstacle_file = 'world_obstacles.txt'
start_goal_file = 'start_goal.txt'

def generate_graph(path_data, nsample, k):
	V, E = [], []
	# sample uniformly until have nsample collision-free nodes.
	xy_min = [0, 0]
	xy_max = [600, 600]
	while len(V) < nsample:
		q = np.random.uniform(low=xy_min, high=xy_max, size=(1,2))[0]
		#print(q)
		if not path_data.contains_point(q):
			V.append(q)
			#print(q)
	return V


#def shortest path():
"""Import from last hw?"""

if __name__ == "__main__":
	fig, ax = plt.subplots()
	path = build_obstacle_course(obstacle_file, ax)
	start, goal = add_start_and_goal(start_goal_file, ax)
	point = (200,250)
	a = path.contains_point(point)
	V = generate_graph(path, 5, 1)
	xs, ys = [], []
	for i in range(5)
	plt.show()
