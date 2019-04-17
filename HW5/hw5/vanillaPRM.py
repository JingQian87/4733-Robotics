"""
	Goal: Build a probabilistic roadmap and visualize it on the environment, along with the shortest path. 
	Steps: 
		0. import map from start_goal.txt & world_obstacles.txt
		1. build graph: sample configurations uniformly and use k-nearest-neighbors to find edges.
		2. graph search (use functions from hw4) 
		*. plot
	Notice: Set the number of samples and number of nearest neighbors as free parameters 
"""

import matplotlib.pyplot as plt 
import scipy.spatial import ConvexHull


def generate_graph(nsample, k):


