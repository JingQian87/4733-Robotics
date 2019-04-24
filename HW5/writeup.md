# Writeup for HW5 Programming Part

Names: Jing Qian & Qing Teng

UNIs: jq2282 & qt2126

------

**Briefly discuss your findings for the parameter values that worked best for the vanilla PRM on the given environment, and provide a screenshot of the result. How does performance of the algorithm improve or worsen as you change the free parameters? Look at factors such as path quality, sampling time (number of discarded samples), graph structure (disconnected components, unnecessary connections), and search efficiency.** 

The parameters are the number of samples $n$ and the number of nearest neighbors $k$. The parameter values that worked best are $n=500$ and $k=5$. Here is a screenshot:

![Figure_8](/Users/cmouse/Documents/Columbia Spring 2019/Robotics/HW5/Figure_8.png)

We evaluate the performance of the algorithm based on path quality (length) and search efficiency (time).

| n    | path length        | Time               |
| ---- | ------------------ | ------------------ |
| 300  | inf                | 1.3193333148956299 |
| 400  | inf                | 1.535979986190796  |
| 500  | 1247.7734005839188 | 1.9075641632080078 |
| 600  | 1369.1835593583519 | 2.341942071914673  |

| k    | path length        | Time               |
| ---- | ------------------ | ------------------ |
| 3    | inf                | 1.203444004058838  |
| 4    | 1344.1791882501227 | 1.523287057876587  |
| 5    | 1247.7734005839188 | 1.9075641632080078 |
| 6    | 1066.330250235014  | 2.356560707092285  |

We also look at the space coverage of some parameter combinations:

(1) $n=300$ and $k=5$

![Figure_9](/Users/cmouse/Documents/Columbia Spring 2019/Robotics/HW5/Figure_9.png)

(2) $n=600$ and $k=5$

![Figure_10](/Users/cmouse/Documents/Columbia Spring 2019/Robotics/HW5/Figure_10.png)

(3) $n=500$ and $k=3$

![Figure_11](/Users/cmouse/Documents/Columbia Spring 2019/Robotics/HW5/Figure_11.png)

(4) $n=500$ and $k=6$

![Figure_12](/Users/cmouse/Documents/Columbia Spring 2019/Robotics/HW5/Figure_12.png)

**Briefly describe your extension to PRM and the improvements you observed, if any. What choices of free parameters did you have and how did you choose them? Include a representative screenshot of a PRM generated with this extended method.** 

We implemented PRM Enhancement by focusing on sampling in regions around nodes with fewer neighbors. For each node $c$, we maintained a weight $w$, which is $\frac{1}{p^2}$, where $p$ is the number of neighbors of $c$ within some predefined distance $d$. We selected nodes to enhance based on their weights, sampled $n$ more nodes within $r$ distance of $c$, and added those nodes to the graph. We choose $m$ nodes to enhance in total.

The free parameters are the predefined distance $d$, the number of nodes to sample $n$, the distance $r$ and the total number of nodes to enhance $m$. I chose them to be: $d=30$, $m=30$, $n=5$ and $r=40$.

Here is a representative screenshot:

![Figure_7](/Users/cmouse/Documents/Columbia Spring 2019/Robotics/HW5/Figure_7.png)

The green edges are generated during the enhancement phase.

**Briefly discuss your findings for the parameter values that worked best for the vanilla RRT on the given environment, and provide a screenshot of the result. How does performance of the algorithm improve or worsen as you change the free parameters? Look at factors such as path quality, sampling time (number of thrown away samples), space coverage (too much or too little overall or in specific regions), and search efficiency.** 

The parameters in my implementation include: $bias$, $d$ (the distance between q_near and q_new, i.e. the step size) and $t$ (used for the goal test: we consider the goal to be reached when there is a collision-free path between goal and tree whose length is no larger than $t$).

The parameter values that worked best for the vanilla RRT are: $bias = .05$, $d = 30$ and $t = 50$. This is a screenshot of the result:

![figure_3](/Users/cmouse/Documents/Columbia Spring 2019/Robotics/HW5/figure_3.jpeg)

The corresponding path length is 1197.62.

We evaluate the performance of the algorithm based on path quality (length), percentage of thrown-away samples and search efficiency (number of iterations taken).

First we look at how $bias$ affects these factors:

| bias | path length | percentage of thrown-away samples | number of iterations |
| ---- | ----------- | --------------------------------- | -------------------- |
| 0.01 | 1228.28     | 85.50%                            | 1317                 |
| 0.05 | 1197.62     | 85.89%                            | 1396                 |
| 0.10 | 1250.03     | 87.71%                            | 1521                 |
| 0.20 | 1520.52     | 89.42%                            | 1598                 |

Next we look at how $d$ affects these factors:

| d    | path length | percentage of thrown-away samples | number of iterations |
| ---- | ----------- | --------------------------------- | -------------------- |
| 20   | 1233.27     | 83.42%                            | 2563                 |
| 30   | 1197.62     | 85.89%                            | 1396                 |
| 40   | 1190.29     | 91.11%                            | 1383                 |
| 50   | 1213.65     | 89.93%                            | 735                  |

Finally we look at how $t$ affects these factors:

| t    | path length | percentage of thrown-away samples | number of iterations |
| ---- | ----------- | --------------------------------- | -------------------- |
| 40   | 1119.50     | 86.79%                            | 1234                 |
| 50   | 1197.62     | 85.89%                            | 1396                 |
| 60   | 1266.40     | 84.61%                            | 1163                 |
| 70   | 1194.13     | 80.88%                            | 617                  |

We also look at the space coverage of some parameter combinations:

(1) $bias=0.20$, $d = 30$ and $t = 50$

![Figure_4](/Users/cmouse/Documents/Columbia Spring 2019/Robotics/HW5/Figure_4.png)

(2) $bias=0.05$, $d = 20$ and $t = 50$

![Figure_5](/Users/cmouse/Documents/Columbia Spring 2019/Robotics/HW5/Figure_5.png)

(3) $bias=0.05$, $d = 50$ and $t = 50$

![Figure_6](/Users/cmouse/Documents/Columbia Spring 2019/Robotics/HW5/Figure_6.png)

**Briefly discuss your choices of when and how often you call the RRT-connect procedure for the bidirectional RRT. Did you notice any decrease in performance if called too rarely or too often? Provide a screenshot of the result of a bidirectional RRT (color the two trees with different colors).**

In the bidirectional RRT, we let both trees grow 10 iterations and then try to connect. It the trees fail to connect, we let them grow 2 iterations (gap = 2) and try to connect again. It is true that the performance would decrease if the procedure called connetion too rarely or too often:

1) if we call the connection procedure too rarely, it will waste time in growing unnecessary edges, like the following figure. In this figure, we set the gap = 20, which means between two connections, there are 20 iterations of tree growing. We could see that part of the two trees overlap, which suggests that they may connect long ago.

![Gap20](/Users/cmouse/Documents/Columbia Spring 2019/Robotics/HW5/hw5/Gap20.png)

2) If we call the connection too often, we will also have a low performance. Like the following figure, we set the gap to 0, which means after one connection failed, we try to connect again. The graph is unbalanced which means that start tree grows a lot and quite randomly while the goal tree grows a little.

![Gap0](/Users/cmouse/Documents/Columbia Spring 2019/Robotics/HW5/hw5/Gap0.png)

3) With our choice of parameters, the generated result seems good.

![Grow10Gap2](/Users/cmouse/Documents/Columbia Spring 2019/Robotics/HW5/hw5/Grow10Gap2.png)