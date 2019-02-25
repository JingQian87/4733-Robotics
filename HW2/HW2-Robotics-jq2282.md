# COMS W4733, HW2 — Jing Qian (jq2282)

## Problem 1







## Problem 2

(a)
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





## Problem 3

(a) From the forward kinematics, we could get the linear velocity Jacobian:
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

From the DH parameter table, we could get homogeneous transformations between frame 0 and $i$. We know that $z_i^0$ is the top three elements from $T_i^0$. Since the first and third joints are revolute and joint 2 is prismatic, the angular velocity Jacobian is:
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


(b) The determination of the three non-zero rows of $J$ is:
$$
\mathrm{det}\begin{bmatrix}
-(d_2+2)s_1-c_1-2s_{13} & c_1 & -2s_{23} \\
 (d_2+2)c_1-s_1+2c_{13} & s_1 & 2c_{13}  \\
1 & 0 & 1
\end{bmatrix}
=-d_2-2
$$
When $d_2 = -2​$, the above determination turns to 0, which means that wrist singularity occurs.$\textcolor{red}{???????????????不确定}​$.



(c) The problem is underconstrained because there are only two desired velocities with three joints. The Jacobian is $2\times3​$:
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

The right pseudo-inverse $J_r^+$ minimizes a cost function in joint velocities: $g(\dot{q},v_d) = \frac{1}{2}(v_d - J\dot{q})^T(v_d - J\dot{q})$.$\textcolor{red}{???????????????不确定}​$.



(d)  All possible solutions to the above problem including the homogeneous solution is: $\dot{q} = J_r^+ v_d + (I-J_r^+J)\dot{q}_0​$. So:
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
$\textcolor{red}{Here不确定！}$ $\dot{q}_0$ is arbitrary solutions to $v_d=J\dot{q}$.



(e) The problem is overconstrained because there are more specifications than DOFs. Jacobian has 6 rows and 3 columns.
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

The left pseudo-inverse tries to minimize the resulting error:  $g(\dot{q},v_d) = \frac{1}{2}(v_d - J\dot{q})^T(v_d - J\dot{q})$.$\textcolor{red}{???????????????不确定}$.



(f) Using the joint solution in the previous part, we could get the actual end effector velocities as:
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

