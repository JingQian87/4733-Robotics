import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull

from utils import read_world_data

obstacles_file = "../data/world_obstacles.txt"
goal_file = "../data/goal.txt"

def placed_robot(vertex):
    """ Returns the coordinates of the four vertices of the robot when placed at vertex """
    # Note: the reference point is the square center and the shape and size of the robot is hardcoded
    x, y = vertex
    return (x - 18, y - 18), (x + 18, y - 18), (x - 18, y + 18), (x + 18, y + 18)

def grow_obstacle(obstacle):
    """ Returns the coordinates of the grown obstacle """
    vertices = []
    for vertex in obstacle:
        vertices += placed_robot(vertex)
    # print(vertices)
    hull = ConvexHull(vertices)
    return [vertices[i] for i in hull.vertices] # hull.vertices: indices

def plot_obstacle(obstacle):
    xs = [vertex[0] for vertex in obstacle]
    ys = [vertex[1] for vertex in obstacle]
    xs.append(xs[0])
    ys.append(ys[0])
    plt.plot(xs, ys, 'r--', lw=2)

def grow_obstacles():
    """ Returns the coordinates of obstacles and goal after growing the obstacles """
    obstacles, goal = read_world_data(obstacles_file, goal_file)
    obstacles_new = []

    for obstacle in obstacles:
        obstacle_new = grow_obstacle(obstacle)
        obstacles_new.append(obstacle_new)
    
    return obstacles_new, goal

# def main():
#     obstacles, _ = read_world_data(obstacles_file, goal_file)
# 
#     for obstacle in obstacles:
#         plot_obstacle(obstacle)
#         obstacle_new = grow_obstacle(obstacle)
#         # print(obstacle_new)
#         plot_obstacle(obstacle_new)
#     
#     plt.show()
# 
# if __name__ == "__main__":
#     main()
