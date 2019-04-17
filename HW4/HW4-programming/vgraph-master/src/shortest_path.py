from collections import defaultdict
from heapq import heapify, heappush, heappop
import matplotlib.pyplot as plt

from gene_vgraph import gene_vgraph

def manhattan_dist(p1, p2):
    """ Returns the manhattan distance between two points """
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** .5

def dijkstra(start, goal, adj_list):
    visited, dist, prev, heap = set(), dict(), dict(), []
    for vertex in adj_list.keys():
        if vertex == start:
            dist[vertex] = 0
            prev[vertex] = None
            heap.append((0, vertex))
        else:
            dist[vertex] = float('inf')
            prev[vertex] = None
            heap.append((float('inf'), vertex))
    heapify(heap)

    while heap:
        _, vertex = heappop(heap)
        if vertex in visited:
            continue
        elif vertex == goal: # found min distance to goal, no need to continue
            break
        visited.add(vertex)
        for neighbor, edge_len in adj_list[vertex]:
            if neighbor in visited:
                continue
            elif dist[vertex] + edge_len < dist[neighbor]:
                dist[neighbor] = dist[vertex] + edge_len
                prev[neighbor] = vertex
                heappush(heap, (dist[neighbor], neighbor))
    
    # Recover the path
    path, curr = [], goal
    while curr:
        path.append(curr)
        curr = prev[curr]
    
    return path[::-1], dist[goal]

def shortest_path(obstacles_file, goal_file):
    """ Return the shortest path from start to goal """
    # edges = [((1, 2), (3, 4)), ((3, 4), (2, 4)), ((1, 2), (5, 6)), ((2, 4), (5, 6))]
    # start, goal = (1, 2), (5, 6)

    edges, goal = gene_vgraph(obstacles_file, goal_file)
    start = (0, 0)
    
    # Compute adjacency list
    adj_list = defaultdict(list)
    for edge in edges:
        edge_len = manhattan_dist(edge[0], edge[1])
        adj_list[edge[0]].append((edge[1], edge_len))
        adj_list[edge[1]].append((edge[0], edge_len))
    # print(adj_list)
    
    return dijkstra(start, goal, adj_list)

def main():
    obstacles_file = "../data/world_obstacles.txt"
    goal_file = "../data/goal.txt"
    path_file = "../data/path.txt"

    # Plotting purposes only
    from grow_obstacles import grow_obstacle, plot_obstacle
    from utils import read_world_data

    obstacles, _ = read_world_data(obstacles_file, goal_file)
    
    for obstacle in obstacles:
        plot_obstacle(obstacle)
        obstacle_new = grow_obstacle(obstacle)
        plot_obstacle(obstacle_new)

    edges, _ = gene_vgraph(obstacles_file, goal_file)
    for i in edges:
        xs = [i[0][0], i[1][0]]
        ys = [i[0][1], i[1][1]]
        plt.plot(xs, ys, 'b', lw=1)

    points, path_len = shortest_path(obstacles_file, goal_file)
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    plt.plot(xs, ys, 'g--', lw=3)
    # plt.show()

    lines = []
    for point in points:
        lines.append("{} {}\n".format(point[0], point[1]))
    with open(path_file, 'w') as f:
        f.writelines(lines)

if __name__ == "__main__":
    main()
