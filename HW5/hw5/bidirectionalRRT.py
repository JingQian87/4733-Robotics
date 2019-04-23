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

from shortest_path import manhattan_dist
from vanillaPRM import edges_obstacles, intersect, new_spath
from visualize_map import *

obstacle_file = 'world_obstacles.txt'
start_goal_file = 'start_goal.txt'

BIAS = .05
D = 30
MAX_ITERS = 10000
T = 50
TimeGap = 3 #Maybe extended

xy_min = [0, 0]
xy_max = [600, 600]

def grow(gpoint, bias, d, V, E):
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

	return V, E


plt.ion() # turn the interactive mode on

fig, ax = plt.subplots()
path = build_obstacle_course(obstacle_file, ax)
start, goal = add_start_and_goal(start_goal_file, ax)

E_obs = edges_obstacles(path)

V_start, E_start, V_goal, E_goal = [], [], [], []

num_iters = 0
success = False

while not success and num_iters < MAX_ITERS:
	if num_iters%TimeGap == 2:
		if connect: 
			break
	V_start, E_start = grow(goal, BIAS, D, V_start, E_start)
	V_goal, E_goal = grow(start, BIAS, D, V_goal, E_goal)
	num_iters += 1

if num_iters >= MAX_ITERS:
	print('Fail to connect!')





