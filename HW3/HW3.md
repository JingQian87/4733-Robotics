# COMS W4733, HW3

### Jing Qian (jq2282)

## PROBLEM 1

**(a)** The coordinates $(x',y',\theta')^T$ could be expressed in terms of the link length $L$ and configuration variables $x, y, \theta,$ and $\phi$:
$$
\begin{bmatrix}
x' \\
y' \\
\theta' 
\end{bmatrix}  =\begin{bmatrix}
x+\frac{L}{2}\cos\theta +\frac{L}{2}\cos(\theta+\phi)\\
y+\frac{L}{2}\sin\theta +\frac{L}{2}\sin(\theta+\phi)\\
\theta+\phi 
\end{bmatrix}
$$
Then 
$$
\dot{x}' = \dot{x} - \frac{L}{2}\dot{\theta}\sin\theta - \frac{L}{2}(\dot{\theta}+\dot{\phi})\sin(\theta+\phi)\\
\dot{y}' = \dot{y} + \frac{L}{2}\dot{\theta}\cos\theta + \frac{L}{2}(\dot{\theta}+\dot{\phi})\cos(\theta+\phi)
$$
So the second link's no-slip constraints $\dot{x}'\sin\theta'-\dot{y}'\cos\theta' =0​$ could be expressed in terms of these variables and their velocity:
$$
\dot{x}\sin(\theta+\phi)-\dot{y}\cos(\theta+\phi) - \dot{\theta}L\cos^2\phi-\frac{L}{2}(\dot{\theta}+\dot{\phi})=0.
$$
Or:
$$
\dot{x}\sin(\theta+\phi)-\dot{y}\cos(\theta+\phi) - \dot{\theta}L\cos^2\frac{\phi}{2}-\dot{\phi}\frac{L}{2}=0.
$$


**(b)** The robot's three constraints could be expressed in Pfaffian form $A^T(q)\dot{q} = 0$:
$$
\begin{bmatrix} 
\sin\theta & -\cos\theta & 0 & 0 & 0 \\
\cos\theta & \sin\theta & 0 & -1 & 0 \\
\sin(\theta+\phi) & -\cos(\theta+\phi) & -\frac{L}{2}(\cos\phi+1)&0&-\frac{L}{2}
\end{bmatrix}
\begin{bmatrix}
\dot{x}\\
\dot{y}\\
\dot{\theta}\\
v\\
\dot{\phi}
\end{bmatrix}=\mathbf{0}
$$
Since there are three constraints and 5 configuration velocities, there are 2 controllable degrees of freedom. Set the two controllable degrees of freedom to $v$ and $\dot{\phi}$,  we have the set of all allowed velocities:

$$
\begin{bmatrix}
\dot{x}\\
\dot{y}\\
\dot{\theta}\\
v\\
\dot{\phi}
\end{bmatrix}=
\begin{bmatrix}
\cos\theta & 0\\
\sin\theta & 0\\
\frac{2\sin\phi}{L(\cos\phi+1)} & -\frac{1}{\cos\phi+1}\\
1&0\\
0&1
\end{bmatrix}
\begin{bmatrix}
v\\
\dot{\phi}
\end{bmatrix}
$$



**(c)** Given $\dot{x}(t)$ and $\dot{y}(t)$, from result of 1.(b), we have $v = \sqrt{\dot{x}^2+\dot{y}^2}$.

Since $\dot{\theta} = \frac{\ddot{y}\dot{x}-\ddot{x}\dot{y}}{\dot{x}^2+\dot{y}^2}​$, we have:
$$
\begin{split}
\dot{\phi} & =\frac{2\sin\phi}{L}v -(\cos\phi+1)\dot{\theta}\\
&=\frac{2\sin\phi}{L}\sqrt{\dot{x}^2+\dot{y}^2} - (\cos\phi+1)\frac{\ddot{y}\dot{x}-\ddot{x}\dot{y}}{\dot{x}^2+\dot{y}^2}
\end{split}
$$
So the input velocities could be expressed as following:
$$
\begin{split}
v &= \sqrt{\dot{x}^2+\dot{y}^2} \\
\dot{\phi} & =\frac{2\sin\phi}{L}\sqrt{\dot{x}^2+\dot{y}^2} - (\cos\phi+1)\frac{\ddot{y}\dot{x}-\ddot{x}\dot{y}}{\dot{x}^2+\dot{y}^2}
\end{split}
$$
Yes, the robot has singularities. From the forward kinematics, the constraint of $\dot{\theta}$ exsits when $\cos\phi+1 \neq 0$, which means when $\phi=-\pi$, singularities occur.



## PROBLEM 2

**(a)** The four ODEs are as following:
$$
\begin{split}
\dot\phi(t) &= 0.3 \sin(t)\\
\dot\theta(t) &= \tan(\phi(t))\\
\dot{x} &= \cos(\theta(t))\\
\dot{y} &= \sin(\theta(t))
\end{split}
$$
Use Python **odeint** to integrate the above ODES numerically, we could have the $x(t)-y(t)$ plot:

![xy0.3](/Users/mac/Desktop/Robotics/HW3/xy0.3.png)



![v](/Users/mac/Desktop/Robotics/HW3/v.png)



**(b)**

![xy0.6](/Users/mac/Desktop/Robotics/HW3/xy0.6.png)

**(c)**

![xy1](/Users/mac/Desktop/Robotics/HW3/xy1.png)