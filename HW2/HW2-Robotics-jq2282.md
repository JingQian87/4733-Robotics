# COMS W4733, HW2 — Jing Qian (jq2282)

## Problem 1

**(a)** There are infinite solutions for this problem if we don't care about the angle. Because there is a laser in the end, like a prismatic joint with infinite range of length. Solving functions with three parameters $\theta_1, \theta_2, d_3$ with only two constraints $p_x$ and $p_y$ will lead to infinite solutions.

No, the number of solutions doesn't depend on $l_1$ or $l_2$.



**(b)** We could model the laser as a third prismatic joint with length $d_3$. So we try to solve $(\theta_1, \theta_2, d_3)$ with equations of $(p_x, p_y, \phi)$:
$$
\left\{
\begin{array}{lr}
\phi = \theta_1+\theta_2 \\
p_x = l_1c_1+(l_2+d_3)c_{12}\\
p_y = l_1s_1+(l_2+d_3)s_{12}
\end{array}
\right.
$$
We could get $\theta_1​$ and $$d_3​$$ from:
$$
\left\{
\begin{array}{lr}
p_x = l_1c_1+(l_2+d_3)c_{\phi}\\
p_y = l_1s_1+(l_2+d_3)s_{\phi}
\end{array}
\right.
$$
Then use $\theta_2 = \phi-\theta_1$ . 



**(c)** The robots' workspace is bound by the angle $\phi$ at $(p_x, p_y)$. For point $p_x^2+p_y^2>l_1^2$, we do tangents from this point to the circle with raidus $l_1$. If $\phi$ points to a direction in the region C, there are solutions. But for regions, A, B and D, there is no solution. 

![IMG_0527](/Users/mac/Desktop/4733-Robotics/HW2/IMG_0527.png)



## Problem 2

**(a)**
$$
\begin{equation}
\begin{split}
J_p(q) &= 
\begin{bmatrix}
\frac{\partial p_x}{\partial q_1} & \frac{\partial p_x}{\partial q_2} & \frac{\partial p_x}{\partial q_3} \\
\frac{\partial p_y}{\partial q_1} & \frac{\partial p_y}{\partial q_2} & \frac{\partial p_y}{\partial q_3}  \\
\frac{\partial p_z}{\partial q_1} & \frac{\partial p_z}{\partial q_2} & \frac{\partial p_z}{\partial q_3} 
\end{bmatrix}\\
&= \begin{bmatrix}
-(L_1+L_2c_2+L_3c_{23})s_1 & -L_2c_1s_2-L_3c_1s_{23} & -L_3c_1s_{23} \\
 (L_1+L_2c_2+L_3c_{23})c_1 & -L_2s_1s_2-L_3s_1s_{23} & -L_3s_1s_{23}  \\
0 & L_2c_2+L_3c_{23} & L_3c_{23}
\end{bmatrix}
\end{split}
\end{equation}
$$


| Link | $a_i$ | $\alpha_i$ | $d_i$ | $\theta_i$ |
| ---- | ----- | ---------- | ----- | ---------- |
| 1    | $L_1$ | 90         | 0     | $\theta_1$ |
| 2    | $L_2$ | 0          | 0     | $\theta_2$ |
| 3    | $L_3$ | 0          | 0     | $\theta_3$ |

From the DH parameter table, we could get homogeneous transformations between frame 0 and $i​$. We know that $z_i^0​$ is the top three elements of the third column from $T_i^0​$. The angular velocity Jacobian is:

$$
\begin{split}
J_0 &= \begin{bmatrix}
z_0^0 & z_1^0 & z_2^0
\end{bmatrix}\\
&= \begin{bmatrix}
0 & s_1 & s_1\\
0 & -c_1 & -c_1\\
1 & 0 & 0
\end{bmatrix}
\end{split}
$$
So the full Jacobian matrix is:
$$
J=\begin{bmatrix}
-(L_1+L_2c_2+L_3c_{23})s_1 & -L_2c_1s_2-L_3c_1s_{23} & -L_3c_1s_{23} \\
 (L_1+L_2c_2+L_3c_{23})c_1 & -L_2s_1s_2-L_3s_1s_{23} & -L_3s_1s_{23}  \\
0 & L_2c_2+L_3c_{23} & L_3c_{23}\\
0 & s_1 & s_1\\
0 & -c_1 & -c_1\\
1 & 0 & 0
\end{bmatrix}
$$


**(b)** The determination of $J_P$ is:
$$
\mathrm{det}(J_P)
=-l_2l_3s_3(l_1+l_2c_2+l_3c_{23})
$$
When $s_3=0$ or $l_1+l_2c_2+l_3c_{23}=0$, $\mathrm{det}(J_p)=0$ and singularities occur.



**(c)** When $\theta_3=0$ or $\pi$, $s_3=0$, elbow singularity occurs.

When $l_1+l_2c_2+l_3c_{23}=0$, shoulder singularity occurs.

![IMG_3157](/Users/mac/Desktop/4733-Robotics/HW2/IMG_3157.png)

## Problem 3

**(a)** From the forward kinematics, we could get the linear velocity Jacobian:
$$
\begin{equation}
\begin{split}
J_p(q) &= 
\begin{bmatrix}
\frac{\partial p_x}{\partial q_1} & \frac{\partial p_x}{\partial q_2} & \frac{\partial p_x}{\partial q_3} \\
\frac{\partial p_y}{\partial q_1} & \frac{\partial p_y}{\partial q_2} & \frac{\partial p_y}{\partial q_3}  \\
\frac{\partial p_z}{\partial q_1} & \frac{\partial p_z}{\partial q_2} & \frac{\partial p_z}{\partial q_3} 
\end{bmatrix}\\
&= \begin{bmatrix}
-(d_2+2)s_1-c_1-2s_{13} & c_1 & -2s_{23} \\
 (d_2+2)c_1-s_1+2c_{13} & s_1 & 2c_{13}  \\
0 & 0 & 0
\end{bmatrix}
\end{split}
\end{equation}
$$
We could get the following DH parameter table:

| Link | $a_i$ | $\alpha_i$ | $d_i$   | $\theta_i$    |
| ---- | ----- | ---------- | ------- | ------------- |
| 1    | 1     | 90         | 0       | $\theta_1$+90 |
| 2    | 0     | -90        | $d_2$+2 | 0             |
| 3    | 2     | 0          | 0       | $\theta_3$-90 |

From the DH parameter table, we could get homogeneous transformations between frame 0 and $i​$. We know that $z_i^0​$ is the top three elements of the third column from $T_i^0​$. Since the first and third joints are revolute and joint 2 is prismatic, the angular velocity Jacobian is:
$$
\begin{split}
J_0 &= \begin{bmatrix}
z_0^0 & 0 & z_2^0
\end{bmatrix}\\
&= \begin{bmatrix}
0 & 0 & 0\\
0 & 0 & 0\\
1 & 0 & 1
\end{bmatrix}
\end{split}
$$
So the full $6\times3​$ Jacobian matrix is:
$$
J =\begin{bmatrix}
-(d_2+2)s_1-c_1-2s_{13} & c_1 & -2s_{23} \\
 (d_2+2)c_1-s_1+2c_{13} & s_1 & 2c_{13}  \\
0 & 0 & 0 \\
0 & 0 & 0\\
0 & 0 & 0\\
1 & 0 & 1
\end{bmatrix}
$$



**(b)** The determination of the three non-zero rows of $J$ is:
$$
\mathrm{det}\begin{bmatrix}
-(d_2+2)s_1-c_1-2s_{13} & c_1 & -2s_{23} \\
 (d_2+2)c_1-s_1+2c_{13} & s_1 & 2c_{13}  \\
1 & 0 & 1
\end{bmatrix}
=-d_2-2
$$
When $d_2 = -2$, the above determination turns to 0, singularities occurs. We couldn't differentiate the contribution of $\theta_1$ and $\theta_3$.



**(c)** The problem is underconstrained because there are only two desired velocities with three joints. The Jacobian is $2\times3$:
$$
J = \begin{bmatrix}
-(d_2+2)s_1-c_1-2s_{13} & c_1 & -2s_{23} \\
 (d_2+2)c_1-s_1+2c_{13} & s_1 & 2c_{13}
\end{bmatrix}
=\begin{bmatrix}
-2-1.5\sqrt{3} & 0.5\sqrt{3} & -\sqrt{3} \\
 0.5+2\sqrt{3} & 0.5 & 1
\end{bmatrix}
$$
We could use the pseudoinverse to numerically find the "best" solution:
$$
J_r^+ =\begin{bmatrix}
-0.015 & 0.216 \\
 0.59 & 0.709 \\
 -0.235 & -0.211
\end{bmatrix}
$$

$$
\dot{q}^* =J_r^+ v_d=\begin{bmatrix}
-0.015 & 0.216 \\
 0.59 & 0.709 \\
 -0.235 & -0.211
\end{bmatrix}
\begin{bmatrix}
-1 \\
2
\end{bmatrix}
=\begin{bmatrix}
0.447\\
 0.828\\
-0.187
\end{bmatrix}
$$

The right pseudo-inverse $J_r^+​$ minimizes a cost function in joint velocities: $g(\dot{q},v_d) = \frac{1}{2}(v_d - J\dot{q})^T(v_d - J\dot{q})​$.



**(d)** All possible solutions to the above problem including the homogeneous solution is: $\dot{q} = J_r^+ v_d + (I-J_r^+J)\dot{q}_0$. So:
$$
\bold{P} =I-J_r^+J =I - \begin{bmatrix}
-0.015 & 0.216 \\
 0.59 & 0.709 \\
 -0.235 & -0.211
\end{bmatrix}
\begin{bmatrix}
-2-1.5\sqrt{3} & 0.5\sqrt{3} & -\sqrt{3} \\
 0.5+2\sqrt{3} & 0.5 & 1
\end{bmatrix} = 
\begin{bmatrix}
 0.075 & -0.095 & -0.242 \\
-0.098 & 0.123 & 0.313 \\
-0.244 & 0.314 & 0.804
\end{bmatrix}
$$
 $\dot{q}_0$ is arbitrary solutions to $v_d=J\dot{q}$.



**(e)** The problem is overconstrained because there are more specifications than DOFs. Jacobian has 6 rows and 3 columns.
$$
J =\begin{bmatrix}
-2-1.5\sqrt{3} & 0.5\sqrt{3} & -\sqrt{3} \\
 0.5+2\sqrt{3} & 0.5 & 1 \\
0 & 0 & 0 \\
0 & 0 & 0\\
0 & 0 & 0\\
1 & 0 & 1
\end{bmatrix}
$$
We could use the pseudoinverse to numerically find the "best" solution:
$$
J_l^+ =\begin{bmatrix}
-0.123 & 0.218 & 0 & 0 & 0 & -0.432 \\
 0.730 & 0.706 & 0 & 0 & 0 & 0.559 \\
 0.123 &-0.218 & 0 & 0 & 0 & 1.432
\end{bmatrix}
$$

$$
\dot{q}^* =J_l^+ v_d=\begin{bmatrix}
-0.123 & 0.218 & 0 & 0 & 0 & -0.432 \\
 0.730 & 0.706 & 0 & 0 & 0 & 0.559 \\
 0.123 &-0.218 & 0 & 0 & 0 & 1.432
\end{bmatrix}
\begin{bmatrix}
-1 \\
2\\
1\\
-3\\
0\\
-2
\end{bmatrix}
=\begin{bmatrix}
 1.423\\
-0.436\\
-3.423
\end{bmatrix}
$$

The left pseudo-inverse tries to minimize the resulting error:  $g(\dot{q},v_d) = \frac{1}{2}(v_d - J\dot{q})^T(v_d - J\dot{q})$.



**(f)** Using the joint solution in the previous part, we could get the actual end effector velocities as:
$$
v_e = J\dot{q}^* =\begin{bmatrix}
-2-1.5\sqrt{3} & 0.5\sqrt{3} & -\sqrt{3} \\
 0.5+2\sqrt{3} & 0.5 & 1 \\
0 & 0 & 0 \\
0 & 0 & 0\\
0 & 0 & 0\\
1 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
 1.423\\
-0.436\\
-3.423
\end{bmatrix}
 = 
 \begin{bmatrix}
-1.001 \\
 2.000 \\
 0\\
 0\\
 0\\
-2.000
\end{bmatrix}
$$
Comparing with the workspace velocities, we could find that $\dot{z},\ w_x$ could not achieve. It is worth mentioning that $w_y$ could not achieve either although the $w_y$ in $v_e$ agrees with $w_y$ given. Because these three components are related with the motion perpenticular to the plane, which could not be achieved by the planar manipulator.

## Problem 4

**(a)** Since the final velocity is 1 rad/s and the deceleration rate is 2 rad/$\rm{s}^2$, if with another $t_m = \frac{1}{2}$s, the velocity at $t=t_f+t_m​$ will be 0 rad/s. Then the modified position, velocity and acceleration profiles will be the same as the LSPD we learnt in class.

Using $t_d$ to denote the time the joint begins to decelerate, we have:
$$
2 = q(t_f) = 0.5 * \ddot{q}_c* t_c^2*2 +  \ddot{q}_c*t_c*(t_d-t_c) - 0.5* \ddot{q}_c*t_m^2=2t_ct_d-0.25
$$

$$
\left\{
\begin{array}{lr}
t_c*t_d=\frac{9}{8} \\
2.5-t_d = t_c
\end{array}
\right.
$$

$$
t_c = \frac{5-\sqrt{7}}{4},\ t_d = \frac{5+\sqrt{7}}{4}
$$

We ignore the reverse solution because $t_c <t_d$.

So we have 
$$
\dot{q}(t_c) = \ddot{q}_ct_c \approx 1.18\ \rm{rad/s},\\
q(t_c) = 0.5 \ddot{q}_c t_c^2 \approx 0.35\ \rm{rad},\\
q(t_d) = 2+0.5*2*0.5^2-q(t_c) \approx 1.9\ \rm{rad}.
$$
The profiles are: ![IMG_1221](/Users/mac/Desktop/IMG_1221.png)



**(b)** Knowing that $\dot{q}_c=1.5$ rad/s and final velocity is $\dot{q}_f = 1$ rad/s, similar to previous part, we could expolate the trajectory to the LSPD we learnt in class. Still use $t_d$ to denote the deceleration time.
$$
\left\{
\begin{array}{lr}
\frac{2-t_d}{t_c} = \frac{1.5-1}{1.5} \\
2 = 0.5\dot{q_c}t_c - (1-(\frac{2}{3})^2)*(0.5\dot{q_c}t_c)+\dot{q_c}(t_d-t_c)
\end{array}
\right.
$$
And we could get the solution:$t_c = 1.2\ \rm{s},\ t_d=1.6\ \rm{s}.$

So we have: $\ddot{q}_c=\dot{q}_c/t_c=1.25\ \rm{rad/s^2}$, $q_c = 0.5\dot{q_c}t_c$ = 0.9 rad, $q(t_d) = q_c+\dot{q}_c(t_d-t_c) = $1.5 rad.

The profiles are:

![IMG_4377](/Users/mac/Desktop/4733-Robotics/HW2/IMG_4377.png)



## Problem 5

**(a)** From the DH parameters, we could get the transformation matrix:
$$
A_1^0 = \begin{bmatrix}
c_1 & -s_1 & 0 & 0 \\
s_1 & c_1 & 0 & 0  \\
0 & 0 & 1 & 0.333 \\
0&0&0&1
\end{bmatrix}, \
A_2^1 = \begin{bmatrix}
c_2&0&-s_2&0 \\
s_2&0&c_2& 0  \\
0&-1&0&0 \\
0&0&0&1
\end{bmatrix},\ 
A_3^2 = \begin{bmatrix}
c_3&0&s_3&0 \\
s_3&0&-c_3& 0  \\
0&1&0&0.316 \\
0&0&0&1
\end{bmatrix}
$$

$$
A_4^3 = \begin{bmatrix}
c_4&0&s_4&0.0825c_4 \\
s_4&0&-c_4&0.0825s_4  \\
0&1&0&0 \\
0&0&0&1
\end{bmatrix}, \
A_5^4 = \begin{bmatrix}
c_5&0&-s_5&-0.0825c_5 \\
s_5&0&c_5&-0.0825s_5\\
0&-1&0&0.384 \\
0&0&0&1
\end{bmatrix},\ 
A_6^5 = \begin{bmatrix}
c_6&0&s_6&0 \\
s_6&0&-c_6& 0  \\
0&1&0&0 \\
0&0&0&1
\end{bmatrix}
$$

$$
A_7^6 = \begin{bmatrix}
c_7&0&s_7&0.088c_7 \\
s_7&0&-c_7&0.088s_7  \\
0&1&0&0 \\
0&0&0&1
\end{bmatrix}
$$

Because of the computation complexity, we calculate the linear velocity Jacobian with $[J_{Pi}] = z_{i-1}^0\times(p_e-p_{i-1})$ and angular velocity Jacobian with $[J_{Oi}] = z_{i-1}^0$. Corresponding code is provided.

