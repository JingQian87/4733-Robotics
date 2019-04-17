"""
This part contains two steps:
a) Generate edges:
	a.1) obstacle edges
	a.2) all dges
b) Use collision checker (intersection script) to remove unwanted edges:
	for i in all_edges:
		for j in obstacle_edges:
			if i==j or i==j[-1]: 
				break # keep all obstacle edges
			elif i and j have common points: 
				continue
			else:
				if intersect:
					remove collisions
"""
import matplotlib.pyplot as plt

from grow_obstacles import grow_obstacles

start = (0, 0)

def ccw(A,B,C):
	return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])

def intersect(A,B,C,D):
	return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def generate_edges():
	# Edges are a tuple whose two elements are starting and ending points.
	obstacles, goal = grow_obstacles()
	obstacle_edges= []
	vertices = []
	# Generate obstacle_edges
	for iob in obstacles:
		for j in range(len(iob)-1):
			obstacle_edges.append((iob[j], iob[j+1]))
			vertices.append(iob[j])
		obstacle_edges.append((iob[-1], iob[0]))
		vertices.append(iob[-1])
	# Generate start_egdes & goal_edges
	all_edges = obstacle_edges[:]
	for i in vertices:
		all_edges.append((start, i))
		all_edges.append((i, goal))
	# Generate edges connecting different obstacles
	for i in range(len(obstacles)-1):
		for j in range(i+1, len(obstacles)):
			for ia in obstacles[i]:
				for jb in obstacles[j]:
					all_edges.append((ia, jb))
	return obstacle_edges, all_edges

def remove_edges(obstacle_edges, all_edges):
	clean_edges = all_edges[:]
	for i in all_edges:
		for j in obstacle_edges:
			# Keep all obstacle edges
			if i==j or i==j[-1]: 
				break 
			# If i and j have common points, let other edges judge.
			elif not set(i).isdisjoint(set(j)):
				continue
			# Delete edges that collide with obstacle edges.
			elif intersect(i[0], i[1], j[0], j[1]):
				clean_edges.remove(i)	
				break

				
	return clean_edges

def gene_vgraph(obstacles_file, goal_file):
	""" API for obtaining generated vgraph """
	from utils import read_world_data
	from grow_obstacles import grow_obstacle, plot_obstacle
	obstacles, goal = read_world_data(obstacles_file, goal_file)

	for obstacle in obstacles:
		obstacle_new = grow_obstacle(obstacle)

	Oedges, Aedges = generate_edges()
	edges = remove_edges(Oedges, Aedges)

	return edges, goal

def main():
	from utils import read_world_data
	from grow_obstacles import grow_obstacle, plot_obstacle
	obstacles_file = "../data/world_obstacles.txt"
	goal_file = "../data/goal.txt"
	obstacles, _ = read_world_data(obstacles_file, goal_file)

	for obstacle in obstacles:
		plot_obstacle(obstacle)
		obstacle_new = grow_obstacle(obstacle)
		# print(obstacle_new)
		plot_obstacle(obstacle_new)

	Oedges, Aedges = generate_edges()
	edges = remove_edges(Oedges, Aedges)
	for i in edges:
		xs = [i[0][0], i[1][0]]
		ys = [i[0][1], i[1][1]]
		plt.plot(xs, ys, 'b', lw=1)

	plt.savefig('Q2.png')
	plt.show()

if __name__ == "__main__":
    main()


