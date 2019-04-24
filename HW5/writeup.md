# Writeup for HW5 Programming Part

Names: Jing Qian & Qing Teng

UNIs: jq2282 & qt2126

**Briefly discuss your findings for the parameter values that worked best for the vanilla PRM on the given environment, and provide a screenshot of the result. How does performance of the algorithm improve or worsen as you change the free parameters? Look at factors such as path quality, sampling time (number of discarded samples), graph structure (disconnected components, unnecessary connections), and search efficiency.** 



**Briefly describe your extension to PRM and the improvements you observed, if any. What choices of free parameters did you have and how did you choose them? Include a representative screenshot of a PRM generated with this extended method.** 

We implemented PRM Enhancement by focusing on sampling in regions around nodes with fewer neighbors. For each node $c$, we maintained a weight $w$, which was the number of neighbors of $c$ within some predefined distance $d$. We selected nodes to enhance based on their weights, sampled $n$ more nodes within $2*d$ distance of $c$, added those nodes to the graph and updated the weights coordinately. We repeat the process for $iter$ times.

The free parameters are the predefined distance $d$, the number of nodes to sample $n$ and the number of iterations $iters$. 

