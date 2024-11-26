# Original paper

This team-work is inspired from the article: [Nonlinear Sliding Mode Control of a Two-Wheeled Mobile](https://discovery.ucl.ac.uk/id/eprint/1551571/1/Spurgeon_authorFinalVersion.pdf)

# Work Description

We primarily reproduced the "Control Design of WMR" section from the paper, utilizing MATLAB for the control design of a two-wheeled mobile robot (WMR) system. A sliding surface was designed for the system, and its stability was analyzed. The tracking problem under reference trajectory constraints was considered, and it was proven that the stability of the sliding mode is asymptotically stable. Additionally, the reachability of the sliding mode was demonstrated.

# Details Explanation

## Model initialization

```
r = 0.0315; % Wheel radius
R = 0.09; % Distance between two wheels
c1 = 0.6; % Sliding surface parameter
eta1 = 1.2; % Reaching gain for sigma1
eta2 = 0.1; % Reaching gain for sigma2

```

$$
\left[\begin{array}{c}
v \\
\omega
\end{array}\right]=\left[\begin{array}{cc}
\frac{r}{2} & \frac{r}{2} \\
\frac{r}{R} & -\frac{r}{R}
\end{array}\right]\left[\begin{array}{l}
\omega_R \\
\omega_L
\end{array}\right] 
$$

## Error analysis

```
xe = (qr(1) - q(1)) * cos(q(3)) + (qr(2) - q(2)) * sin(q(3));
ye = -(qr(1) - q(1)) * sin(q(3)) + (qr(2) - q(2)) * cos(q(3));
theta_e = qr(3) - q(3);
qe = [xe; ye; theta_e];

```

The github's markdown is not that full to show the code, so I show the equation in the following picture.

![eq2.png](/figures/eq2.png)

$$
\begin{gathered}
\mathbf{q}_e = \begin{bmatrix}
x_e \\
y_e \\
\theta_e
\end{bmatrix} = T(\mathbf{q})\left(\mathbf{q}_r - \mathbf{q}\right), \\

T(\mathbf{q}) = \begin{bmatrix}
\cos \theta & \sin \theta & 0 \\
-\sin \theta & \cos \theta & 0 \\
0 & 0 & 1
\end{bmatrix}
\end{gathered}
$$

## Nonlinear silding mode control method

Define the input $u, η1,η2$ are positive reaching gains, $J_n$ is the Jacobian Matrix of the sliding functions.

```
sigma1 = c1 * theta_e + atan(ye);
    sigma2 = xe;
    Jn = [0, 1/(1 + ye^2), 0; 1, 0, 0];
    E = [0, -(c1 + xe/(1 + ye^2)); -1, ye];
    u = -inv(E) * (Jn * [vr_ref * cos(theta_e);vr_ref * sin(theta_e);wr_ref] + [eta1 * sign(sigma1); eta2 * sign(sigma2)]);
    v = u(1);
    w = u(2);
```

![eq3.png](/figures/eq3.png)

$$
\begin{aligned}
\sigma &= \begin{bmatrix}
\sigma_1 \\
\sigma_2
\end{bmatrix} = \begin{bmatrix}
c_1 \theta_e + \tan^{-1}(y_e) \\
x_e
\end{bmatrix} \\
\\
\mathbf{u} &= -\mathbf{E}^{-1} \left[ \mathbf{J}_n \begin{bmatrix}
v_r \cos \theta_e \\
v_r \sin \theta_e \\
\omega_r
\end{bmatrix} + \begin{bmatrix}
\eta_1 \operatorname{sgn}(\sigma_1) \\
\eta_2 \operatorname{sgn}(\sigma_2)
\end{bmatrix} \right] \\
\\
\mathbf{J}_n &= \begin{bmatrix}
0 & \frac{1}{1+y_e^2} & 0 \\
1 & 0 & 0
\end{bmatrix}, \quad \mathbf{E} = \begin{bmatrix}
0 & -\left(c_1+\frac{x_e}{1+y_e^2}\right) \\
-1 & y_e
\end{bmatrix}
\end{aligned}
$$


# Experiments
The main simulation results with a circle reference trajectory with initial condition $q_r(0, 0, π4 )$, reference control pair $v_r = 0.25, ω_r = 0.5$ and initial posture of the actual robot $q(−0.2, −0.3, 0)$ are shown in below figures.



![tracking errors.jpg](figures/tracking_errors.jpg)




![trajectory tracking.jpg](figures/trajectory_tracking.jpg)

