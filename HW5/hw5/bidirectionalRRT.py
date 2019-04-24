"""
	Build bidirectional RRT based on the unidirectional RRT.
	Steps:
	1. def grow(start, goal, bias, V, E)
	2. while not connected and #iter <= MAX_ITERS:
		 for i in range(gap):
		   grow for start
		   grow for goal
		   #iter += 1
		 try connect 
	3. if #iter > MAX_ITERS: 
	     print("no solution")
	   else:
	   	 shortest path from vanillaPRM 
"""
from random import choices
import matplotlib.pyplot as plt
import numpy as np
from shortest_path import manhattan_dist
from vanillaPRM import edges_obstacles, intersect, new_spath
from visualize_map import *
import os

obstacle_file = 'world_obstacles.txt'
start_goal_file = 'start_goal.txt'

BIAS = .05
D = 30
MAX_ITERS = 10000
T = 50
INITIAL_GROW = 10 
GROW_GAP = 2

xy_min = [0, 0]
xy_max = [600, 600]

def grow(gpoint, bias, d, V, E, flag=0):
	while True:
		q_rand = np.random.uniform(low=xy_min, high=xy_max, size=(1,2))[0]
		while path.contains_point(q_rand):
			q_rand = np.random.uniform(low=xy_min, high=xy_max, size=(1,2))[0]
		q_rand = tuple(q_rand.tolist())

		# with (1 - bias) prob
		probs = [1 - bias, bias]
		q_rand = choices(population=[q_rand, goal], weights=probs, k=1)[0]

		# Find closest point in tree q_near
		mindist, q_near = float('inf'), None
		for node in V:
			dist = manhattan_dist(q_rand, node)
			#print(node, dist)
			if dist < mindist:
				mindist, q_near = dist, node
		# print(q_near)

		if mindist < d: # q_new will be farther than q_rand,
			# and no way to guarantee that (q_near, q_new) will be collision-free
			continue

		#print(q_near, q_rand)
		# find path and check collisions
		for l in E_obs:
			#print(l)
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
			if flag == 0:
				epatch = patches.PathPatch(ep, facecolor='None', edgecolor='xkcd:green')
			else:
				epatch = patches.PathPatch(ep, facecolor='None', edgecolor='xkcd:red')
			ax.add_patch(epatch)
			plt.draw()
			plt.pause(.05)
			break
	return V, E


plt.ion() # turn the interactive mode on

fig, ax = plt.subplots()
path = build_obstacle_course(obstacle_file, ax)
start, goal = add_start_and_goal(start_goal_file, ax)

E_obs = edges_obstacles(path)

V_start, E_start, V_goal, E_goal = [start], [], [goal], []

num_iters = 0

for i in range(INITIAL_GROW):
	#print(num_iters)
	V_start, E_start = grow(goal, BIAS, D, V_start, E_start, 0)
	V_goal, E_goal = grow(start, BIAS, D, V_goal, E_goal, 1)
	num_iters += 1

def expand(q1new, flag = 1):
	"""
	Returns:
		1.If connected: return 1
		2.If not connected but expand, call expand again
		3.If not connected and not expand, return None
	"""
	if flag == 1:
		V2 = V_goal
	else:
		V2 = V_start
	# Find closest point in tree q_near
	mindist, q_near = float('inf'), None
	for node in V2:
		dist = manhattan_dist(q1new, node)
		if dist < mindist:
			mindist, q_near = dist, node

	for l in E_obs:
		if intersect(q1new, q_near, l[0], l[1]):
			break
	else: #success
		print('connected!')
		E_goal.append((q1new, q_near))
		E_start.append((q_near, q1new))

		ep = Path((q_near, q1_new))
		epatch = patches.PathPatch(ep, facecolor='None', edgecolor='xkcd:green')
		ax.add_patch(epatch)
		plt.draw()
		plt.pause(.05)
		return 1

	if mindist < D: # not ready for connection
		return None
	else:
		scale = float(D) / mindist
		q2new_x = q_near[0] + (q1_new[0] - q_near[0]) * scale
		q2new_y = q_near[1] + (q1_new[1] - q_near[1]) * scale
		q2new = (q2new_x, q2new_y)
		for l in E_obs:
			if intersect(q_near, q2new, l[0], l[1]):
				return None #no expansion
		else:
			ep = Path((q_near, q2new))
			if flag == 1:
				V_goal.append(q2new)
				E_goal.append((q_near, q2new))
				epatch = patches.PathPatch(ep, facecolor='None', edgecolor='xkcd:red')
			else:
				V_start.append(q2new)
				E_start.append((q_near, q2new))
				epatch = patches.PathPatch(ep, facecolor='None', edgecolor='xkcd:green')
			ax.add_patch(epatch)
			plt.draw()
			plt.pause(.05)
			return expand(q2new, 1-flag)

while num_iters < MAX_ITERS:
	#print(num_iters)
	num_iters += 1
	V_start, E_start = grow(goal, BIAS, D, V_start, E_start, 0)
	q1_new = V_start[-1]
	tmp = expand(q1_new, 1)
	if tmp: 
		break
	print("Not ready to connect, grow again!")
	for i in range(GROW_GAP):
		num_iters += 1
		#print(num_iters)
		V_start, E_start = grow(goal, BIAS, D, V_start, E_start, 0)
		V_goal, E_goal = grow(start, BIAS, D, V_goal, E_goal, 1)	
else:
	print("There is no path from start to goal")
	os._exit(0)

V_all = V_start + V_goal
E_all = E_start + E_goal

points, path_len = new_spath(start, goal, E_all)
xs = [point[0] for point in points]
ys = [point[1] for point in points]
plt.plot(xs, ys, 'k--', lw=2)
print("Path length: {}".format(path_len))

plt.ioff()
plt.show()










