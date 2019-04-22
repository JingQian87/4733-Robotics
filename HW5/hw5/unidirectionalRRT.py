"""
    Builds a unidirectional RRT.

    Steps:
    1. Sample random (valid) configuration q_rand
        a. with (1 - bias) prob
    2. Find closest point in tree q_near
        a. find path and check collisions
    3. If collision-free, connect q_near to q_new that is a distance d away
"""

from random import choices
import matplotlib.pyplot as plt
import numpy as np

from shortest_path import manhattan_dist
from vanillaPRM import edges_obstacles, intersect
from visualize_map import *

obstacle_file = 'world_obstacles.txt'
start_goal_file = 'start_goal.txt'

BIAS = .05
D = 20

if __name__ == "__main__":
    fig, ax = plt.subplots()
    path = build_obstacle_course(obstacle_file, ax)
    start, goal = add_start_and_goal(start_goal_file, ax)

    E_obs = edges_obstacles(path) # Array of obstacle edges

    V, E = [], [] # initialize empty tree
    V.append(start) # add start to tree

    xy_min = [0, 0]
    xy_max = [600, 600]

    num_tries = 0

    while True:
        num_tries += 1
        # Sample random configuration q_rand
        q_rand = np.random.uniform(low=xy_min, high=xy_max, size=(1,2))[0]
        while path.contains_point(q_rand):
            q_rand = np.random.uniform(low=xy_min, high=xy_max, size=(1,2))[0]

        q_rand = tuple(q_rand.tolist())
        # print(q_rand)

        # with (1 - bias) prob
        probs = [1 - BIAS, BIAS]
        q_rand = choices(population=[q_rand, goal], weights=probs, k=1)[0]
        # print(q_rand)

        # Find closest point in tree q_near
        mindist, q_near = float('inf'), None
        for node in V:
            dist = manhattan_dist(q_rand, node)
            if dist < mindist:
                mindist, q_near = dist, node
        # print(q_near)

        # find path and check collisions
        for l in E_obs:
            if intersect(q_near, q_rand, l[0], l[1]):
                break
        else:
            print("Collision-free path from q_near to q_rand")
            print(num_tries)
            break
