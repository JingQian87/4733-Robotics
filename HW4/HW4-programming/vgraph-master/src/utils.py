def read_world_data(obstacles_file, goal_file):
    """ Returns coordinates of obstacles and goal read from files """
    obstacles = [] # list of list of (x, y) coordinates
    with open(obstacles_file, 'r') as f:
        lines = f.readlines()
        obstacles_cnt, i = int(lines[0]), 1
        # print(obstacles_cnt)

        for _ in range(obstacles_cnt):
            obstacle = []
            vertices_cnt = int(lines[i])
            i += 1

            for _ in range(vertices_cnt):
                x, y = lines[i].split(' ')
                obstacle.append((float(x), float(y)))
                i += 1
            
            obstacles.append(obstacle)
    # print(obstacles)

    goal = (0, 0)
    with open(goal_file, 'r') as f:
        lines = f.readlines()
        x, y = lines[0].split(' ')
        goal = (float(x), float(y))
    # print(goal)
    
    return obstacles, goal

# read_world_data("../data/world_obstacles.txt", "../data/goal.txt")
