## Video Link:
vgraph.mov in the link https://drive.google.com/drive/folders/1ufOg04xsXjYbcGkTSoer-4hwJ8xIw7Cl?usp=sharing

## Assumptions

We are assuming that the robot is a 36 x 36 square and the reference point is the square center.

## Usage

### Task 1: Grow obstacles.

`python src/grow_obstacles.py`

By default, this reads data from `data/world_obstacles.txt` and `data/goal.txt`, grows the obstacles, and plots the obstacles (before and after) with `matplotlib`. You can see the result at `results/Q1.png`.

### Task 2: Create visibility graph.

`python src/gene_vgraph.py`

This generates the visibility graph and plots the graph with `matplotlib`. You can see the result at `results/Q2.png`.

### Task 3: Compute shortest path.

`python src/shortest_path.py`

This computes the shortest path in the visibility graph with Dijkstra's algorithm and plots the path with `matplotlib`. You can see the result at `results/Q3.png`.

### Task 4: Move the robot.

First ensure that you have the environment set up and ROS running.

`python markers.py`

This visualizes the results from tasks 1, 2 and 3 in RViz.

`python move_robot.py`

This moves the robot along the shortest path. Way to go!

## Implementation

Our implementation is fairly straightforward. 

One design choice worthing noting is for removing edges that collide with obstacles. We wrote our own collision checker and the basic idea is: the edge collides with obstacles when it intersects with at least one of the obstacle edges. The intersection examination of line segments is based on a trick we found on the Internet, which corresponds to the intersect function in gene_vgraph.py. One tricky part is, we had to work out the details of what to do when an edge shared an endpoint with the edge of an obstacle. We found that if an edge only shared an endpoint with the obstacle and no other intersects, it is fine to keep it; if the edge shared an endpoint with the edge of an obstacle actually collides with the obstacle, it will intersect with other edges of this specific obstacle because we exclude the totally inside edges like diagonals in the first generating step.

While applying code from `odom_out_and_back.py` was mostly sufficient, we had to modify it a little:
First, we generate loops of movement and set each point in the path as a immediate goal in each loop. 
Second, since the robot is not moving right like it does in the `odom_out_and_back.py`, for each movement, we have to rotate the robot to the right goal direction first and then translate it to reduce the distance. 
Third, the `goal_angle`. In `odom_out_and_back.py`, when the robot started to move, its rotation was always zero. But this is not true in our case, because the robot is moving between several points. So we had to modify `goal_angle` to `goal_angle - rotation`.

As we can see, the robot mainly heads directly to the goal, but sways left at first due to the first obstacle.
