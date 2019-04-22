"""
    Builds a unidirectional RRT.

    Steps:
    1. Sample random (valid) configuration q_rand
        a. with (1 - bias) prob
    2. Find closest point in tree q_near
        a. find path and check collisions
    3. If collision-free, connect q_near to q_new that is a distance d away
    4. Repeat until reach goal
        a. goal test: collision-free path between goal and tree is smaller than
            some threshold t
"""

from random import choices
import matplotlib.pyplot as plt
import numpy as np

from shortest_path import manhattan_dist
from vanillaPRM import edges_obstacles, intersect, new_spath
from visualize_map import *

obstacle_file = 'world_obstacles.txt'
start_goal_file = 'start_goal.txt'

BIAS = .05
D = 30
MAX_ITERS = 10000
T = 50

def unidirectionalRRT(start, goal, path, xy_min, xy_max, 
    bias = .05, d = 20, max_iters = 10000, t = 50):

    V, E = [], [] # initialize empty tree
    V.append(start) # add start to tree

    E_obs = edges_obstacles(path) # Array of obstacle edges

    num_iters = 0
    success = False

    while not success and num_iters < max_iters:
        num_iters += 1
        # Sample random configuration q_rand
        q_rand = np.random.uniform(low=xy_min, high=xy_max, size=(1,2))[0]
        while path.contains_point(q_rand):
            q_rand = np.random.uniform(low=xy_min, high=xy_max, size=(1,2))[0]

        q_rand = tuple(q_rand.tolist())
        # print(q_rand)

        # with (1 - bias) prob
        probs = [1 - bias, bias]
        q_rand = choices(population=[q_rand, goal], weights=probs, k=1)[0]
        # print(q_rand)

        # Find closest point in tree q_near
        mindist, q_near = float('inf'), None
        for node in V:
            dist = manhattan_dist(q_rand, node)
            if dist < mindist:
                mindist, q_near = dist, node
        # print(q_near)

        if mindist < d: # q_new will be farther than q_rand,
            # and no way to guarantee that (q_near, q_new) will be collision-free
            continue

        # find path and check collisions
        for l in E_obs:
            if intersect(q_near, q_rand, l[0], l[1]):
                break
        else:
            # print("Collision-free path from q_near to q_rand")
            # connect q_near to q_new that is a distance d away
            scale = float(d) / mindist
            q_new_x = q_near[0] + (q_rand[0] - q_near[0]) * scale
            q_new_y = q_near[1] + (q_rand[1] - q_near[1]) * scale
            q_new = (q_new_x, q_new_y)
            # print(q_new, q_near, q_rand)

            V.append(q_new)
            E.append((q_near, q_new))
            
            ep = Path((q_near, q_new))
            epatch = patches.PathPatch(ep, facecolor='None', edgecolor='xkcd:green')
            ax.add_patch(epatch)
            plt.draw()
            plt.pause(.2)

            # goal test
            # only need to test on newly added nodes!
            for l in E_obs:
                if intersect(q_new, goal, l[0], l[1]):
                    break
            else:
                E.append((q_new, goal))
                if manhattan_dist(q_new, goal) <= t:
                    print("Collision-free path to goal within threshold t!")
                    print("Found after {} iterations...".format(num_iters))
                    success = True

    V.append(goal)
    return V, E

if __name__ == "__main__":
    plt.ion() # turn the interactive mode on

    fig, ax = plt.subplots()
    path = build_obstacle_course(obstacle_file, ax)
    start, goal = add_start_and_goal(start_goal_file, ax)

    xy_min = [0, 0]
    xy_max = [600, 600]

    _, E = unidirectionalRRT(start, goal, path, xy_min, xy_max, 
        bias = BIAS, d = D, max_iters = MAX_ITERS, t = T)
    
    # for e in E:
    #     ep = Path(e)
    #     epatch = patches.PathPatch(ep, facecolor='None', edgecolor='xkcd:green')
    #     ax.add_patch(epatch)
    
    points, path_len = new_spath(start, goal, E)
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    plt.plot(xs, ys, 'b--', lw=1)
    print("Path length: {}".format(path_len))
    
    plt.ioff()
    plt.show()
