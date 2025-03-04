# Reach Avoid Stay Problem
Final Project for AMATH 455: Control Theory.

## Method 1. Mixed Integer Linear Programming

Start with Problem Formation:

Define $N$ as the horizontal variable (time to reach the terminal set), which we will optimize, $\gamma$ as our choice of weight which we may use to balance our cost of control and terminal time.
$$\begin{equation} min N + \gamma\sum^n_{k=0}||u_k|| \end{equation}$$
$s.t.$
$$\begin{equation}\tag{IC}  x_0 = x_{init} \end{equation}$$
$$\begin{equation}\tag{DE}  x_{k+1} = Ax_k+Bu_k\end{equation}$$
$$\begin{equation}\tag{BC1}  x_{k+1} \in \mathcal{X} \end{equation}$$
$$\begin{equation}\tag{BC2}  u_k \in \mathcal{U} \end{equation}$$
$$\begin{equation}\tag{Obstacle Condition}  x_{k+1} \notin \mathcal{X}_U^j, 1 \leq j \leq N_o \end{equation}$$
$$\begin{equation}\tag{Target Condition}  x_{k+1} \in \mathcal{X}_T \end{equation}$$

Where
$$0\leq k \leq N $$
$$ A = 
\begin{bmatrix} 
1 & \tau & 0 & 0\\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & \tau \\
0 & 0 & 0 & 1
\end{bmatrix},
B = 
\begin{bmatrix} 
0.5\tau^2 & 0\\
\tau & 0 \\
0 & 0.5\tau^2 \\
0 & \tau
\end{bmatrix}
$$

One way of formulating the inequalities so it becomes a MPC-MILP (Model Predictive Control Mixed Integer Linear Programming) and can be solved using the algorithms presented in Schouwenaars et al (2001), is to use binary variables.

Consider the obstacles set as the intersection of 4 half planes:
$$\mathcal{X}^j_U = \{p_jCx \leq q_j\}$$
where $p_j$ denotes the constant matrix in the left hand side of the half-plane inequalities defining obstacle $\mathcal{X}^j_U$, and $q_j$ denotes the right hand side of the half plane inequalities, $C$ denotes the matrix that extracts position information from the state vector $x$.

E.g. $\mathcal{X}^1_U$, defined by the lower left extreme [1, 1] and upper right extreme [2, 2] can be represented as:
$$
\mathcal{X}^1_U = \{p_1Cx \leq q_1 \}
$$
where
$$ C= 
\begin{bmatrix} 
1 & 0 & 0 & 0\\
0 & 0 & 1 & 0 \\
\end{bmatrix}
$$
$$
p_1 = 
\begin{bmatrix} 
-1 & 0\\
1 & 0 \\
0 & -1 \\
0 & 1
\end{bmatrix}
$$
$$
q_1 = 
\begin{bmatrix} 
-1\\
2 \\
-1 \\
2
\end{bmatrix}
$$

To ensure the inequality above would never hold for any $x_k$, we use big-M (a large number $M$ as a penalty for entering the obstacle) and binary variables $b_j^{obs} = \{1, 0\}$ for each obstacles. 1 if $x_k$ is outside obstacle j, and 0 if $x_k$ is inside. We also introduced $b_i^{hor}$ to keep track of obstacles we have passed horizontally (=1 if passed the obstacle, =0 if not yet pass) and a small buffer $\epsilon > 0$ to ensure $x_k$ does not graze the obstacle:
$$p_j^{obs}Cx \leq q_j^{obs} + M(1 - b_j^{obs}) - \epsilon + M\sum^{k-1}_{i=0} b^{hor}_i$$

## References
-   Afonso RJM, Maximo MROA, Galvão RKH. Task allocation and trajectory planning for multiple agents in the presence of obstacle and connectivity constraints with mixed-integer linear programming. Int J Robust Nonlinear Control. 2020; 30: 5464–5491. https://doi.org/10.1002/rnc.5092

-   SchouwenaarsT, MoorDB, FeronE, HowJP. Mixed integer programming for multi-vehicle path planning. Paper presented at: Proceedings of the 2001 European Control Conference (ECC). Proceedings of the European Control Conference (ECC); 2001:2603-2608; Porto, Portugal. 

