# Writeup for HW5 Programming Part

Names: Jing Qian & Qing Teng

UNIs: jq2282 & qt2126

**Briefly discuss your findings for the parameter values that worked best for the vanilla PRM on the given environment, and provide a screenshot of the result. How does performance of the algorithm improve or worsen as you change the free parameters? Look at factors such as path quality, sampling time (number of discarded samples), graph structure (disconnected components, unnecessary connections), and search efficiency.** 



**Briefly describe your extension to PRM and the improvements you observed, if any. What choices of free parameters did you have and how did you choose them? Include a representative screenshot of a PRM generated with this extended method.** 

We implemented PRM Enhancement by focusing on sampling in regions around nodes with fewer neighbors. For each node $c$, we maintained a weight $w$, which was the number of neighbors of $c$ within some predefined distance $d$. We selected nodes to enhance based on their weights, sampled $n$ more nodes within $2*d$ distance of $c$, added those nodes to the graph and updated the weights coordinately. We repeat the process for $iter$ times.

The free parameters are the predefined distance $d$, the number of nodes to sample $n$ and the number of iterations $iters$. 

**Briefly discuss your choices of when and how often you call the RRT-connect procedure for the bidirectional RRT. Did you notice any decrease in performance if called too rarely or too often? Provide a screenshot of the result of a bidirectional RRT (color the two trees with different colors).**

In the bidirectional RRT, we let both trees grow 10 iterations and then try to connect. It the trees fail to connect, we let them grow 2 iterations (gap = 2) and try to connect again. It is true that the performance would decrease if the procedure called connetion too rarely or too often:

1) if we call the connection procedure too rarely, it will waste time in growing unnecessary edges, like the following figure. In this figure, we set the gap = 20, which means between two connections, there are 20 iterations of tree growing. We could see that part of the two trees overlap, which suggests that they may connect long ago.

![Gap20](/Users/mac/Desktop/Robotics/HW5/hw5/Gap20.png)

2) If we call the connection too often, we will also have a low performance. Like the following figure, we set the gap to 0, which means after one connection failed, we try to connect again. The graph is unbalanced which means that start tree grows a lot and quite randomly while the goal tree grows a little.

![Gap0](/Users/mac/Desktop/Robotics/HW5/hw5/Gap0.png)

3) With our choice of parameters, the generated result seems good.

![Grow10Gap2](/Users/mac/Desktop/Robotics/HW5/hw5/Grow10Gap2.png)