
# Main Results: Predefined-Time DSC Design

<a id="sec:main_results"></a>
本节针对第 2 节建立的两自由度柔性关节机械臂，给出预定义时间动态面控制
(dynamic surface control, DSC) 设计。设计目标是在不精确知道连杆动力学、
执行器死区和滞环参数的情况下，使连杆侧跟踪误差在用户预先指定的时间
$T_c$ 内进入给定残差集。控制器由四部分组成：预定义时间双幂次误差反馈、
动态面滤波器、每个递推步骤仅含一个标量的最小参数学习器，以及死区--滞环
复合补偿器。为便于表示，以下推导首先按关节展开，所有关节间耦合项均并入
相应的集总不确定性；向量形式在[本节](#subsec:closed_loop)给出。

## 3.1 递推形式与误差面定义

<a id="subsec:recursive_form"></a>
令
$$
 x_{i1}=q_i,\quad x_{i2}=\dot q_i,\quad
 x_{i3}=\theta_i,\quad x_{i4}=\dot\theta_i,\qquad i=1,2 .

 \tag{35}
$$
<a id="eq:component_states"></a>

由式 (1)--(2)可将每个关节写成如下严格反馈的集总形式：
$$
\begin{aligned}
 \dot x_{i1}&=x_{i2},\\
 \dot x_{i2}&=f_{i2}(x,t)+g_{i2}(x,t)x_{i3}+d_{i2}(t),\\
 \dot x_{i3}&=x_{i4},\\
 \dot x_{i4}&=f_{i4}(x,t)+g_{i4}(x,t)\tau_{a,i}+d_{i4}(t),
\end{aligned}

 \tag{36}
$$
<a id="eq:strict_feedback"></a>

其中，$f_{i2}$ 和 $f_{i4}$ 包含重力、科氏力、离心力、摩擦、弹簧耦合及
其余关节的交叉项，$d_{i2}$ 和 $d_{i4}$ 表示外部扰动与未建模动态。对于
标准的输入--输出正则化（或刚度矩阵具有对角占优性质）的柔性关节模型，
控制方向满足
$$
 0<\underline g_i\leq g_{i\ell}(x,t)\leq\overline g_i,
 \qquad \ell\in\{2,4\},

 \tag{37}
$$
<a id="eq:gain_bounds"></a>

其中上下界不要求在线已知。若保留完整的 $M^{-1}(q)K$ 交叉耦合，则将其
非对角项并入 $f_{i2}$，并以相同的正则化变换得到[式](#eq:strict_feedback)。
[式](#eq:strict_feedback) 并不改变原系统，
而是把可计算的标称项与匹配不确定项分开，以便应用假设 4。

定义位置误差面
$$
 s_{i1}=x_{i1}-q_{di} .

 \tag{38}
$$
<a id="eq:s_i1"></a>

对任意 $r\in\mathbb{R}$，记
$$
 \psi_{p}(r)=|r|^{p}\operatorname{sgn}(r),\qquad
 \psi_{q}(r)=|r|^{q}\operatorname{sgn}(r),

 \tag{39}
$$
<a id="eq:power_maps"></a>

其中 $0<p<1$ 且 $q>1$。选取正数 $k_{i1p}$、$k_{i1q}$，并定义第一个
虚拟控制量
$$
 \alpha_{i1}=\dot q_{di}
 -k_{i1p}\psi_p(s_{i1})
 -k_{i1q}\psi_q(s_{i1}).

 \tag{40}
$$
<a id="eq:alpha_i1"></a>

由此得到
$$
 \dot s_{i1}=s_{i2}+y_{i1}
 -k_{i1p}\psi_p(s_{i1})-k_{i1q}\psi_q(s_{i1}),

 \tag{41}
$$
<a id="eq:s_i1_dot"></a>

其中 $\bar\alpha_{i1}$ 为[式](#eq:filter_general) 定义的滤波信号，
$s_{i2}=x_{i2}-\bar\alpha_{i1}$，$y_{i1}=\bar\alpha_{i1}-\alpha_{i1}$。

## 3.2 动态面滤波与预定义时间虚拟控制

<a id="subsec:virtual_controls"></a>
对 $j=1,2,3$，虚拟控制量 $\alpha_{ij}$ 通过
$$
 \tau_{ij}\dot{\bar\alpha}_{ij}+\bar\alpha_{ij}=\alpha_{ij},
 \qquad \bar\alpha_{ij}(0)=\alpha_{ij}(0),
 \qquad \tau_{ij}>0

 \tag{42}
$$
<a id="eq:filter_general"></a>

进行滤波，并令 $y_{ij}=\bar\alpha_{ij}-\alpha_{ij}$。于是
$$
 \dot y_{ij}=-\tau_{ij}^{-1}y_{ij}-\dot\alpha_{ij} .

 \tag{43}
$$
<a id="eq:filter_error_dynamic"></a>

本研究采用 $\tau_{ij}=0.01\,\mathrm{s}$。滤波器只需对虚拟控制量进行一次
动态实现，因此避免了传统反步法中对 $\alpha_{ij}$ 反复解析微分。

第二个误差面及其虚拟控制量定义为
$$
 s_{i2}=x_{i2}-\bar\alpha_{i1},

 \tag{44}
$$
<a id="eq:s_i2"></a>

$$
\begin{aligned}
 \alpha_{i2}={}&\frac{1}{\hat g_{i2}}
 \bigl[-f_{i2}^{0}(x,t)-k_{i2p}\psi_p(s_{i2})
 -k_{i2q}\psi_q(s_{i2})\\
 &\hspace{13mm}-s_{i1}+\dot{\bar\alpha}_{i1}
 -\hat\vartheta_{i2}\xi_{i2}(\chi_{i2})
 -\rho_{i2}\operatorname{sat}(s_{i2}/\epsilon_{i2})\bigr],
\end{aligned}

 \tag{45}
$$
<a id="eq:alpha_i2"></a>

其中 $f_{i2}^{0}$ 是由名义参数计算的连杆侧动力学，$\hat g_{i2}>0$ 为
控制方向估计值，$\chi_{i2}$ 是由 $x$、$q_d$ 及滤波信号构成的回归向量，
$\xi_{i2}(\cdot)$ 为已知基函数，$\rho_{i2}>0$ 和 $\epsilon_{i2}>0$ 分别为
鲁棒增益和边界层厚度。未知项用最小参数学习器逼近：
$$
 \varphi_{i2}(\chi_{i2})
 =\vartheta_{i2}^{*}\xi_{i2}(\chi_{i2})+\varepsilon_{i2}(\chi_{i2}),
 \qquad |\varepsilon_{i2}|\leq\bar\varepsilon_{i2}.

 \tag{46}
$$
<a id="eq:mlp_i2"></a>


第三个误差面和虚拟电机速度控制量为
$$
 s_{i3}=x_{i3}-\bar\alpha_{i2},

 \tag{47}
$$
<a id="eq:s_i3"></a>

$$
 \alpha_{i3}=-k_{i3p}\psi_p(s_{i3})
 -k_{i3q}\psi_q(s_{i3})-s_{i2}
 +\dot{\bar\alpha}_{i2}.

 \tag{48}
$$
<a id="eq:alpha_i3"></a>

[式](#eq:alpha_i3) 中的 $\dot{\bar\alpha}_{i2}$ 由滤波器状态方程
直接计算，不需要对 $\alpha_{i2}$ 作数值微分。

最后一个误差面为
$$
 s_{i4}=x_{i4}-\bar\alpha_{i3} .

 \tag{49}
$$
<a id="eq:s_i4"></a>

记电机侧标称动力学为
$$
 f_{i4}^{0}(x,t)=J_i^{-1}\left[-F_{\theta i}x_{i4}
 +K_i(x_{i1}-x_{i3})\right],

 \tag{50}
$$
<a id="eq:f_i4_nominal"></a>

并令 $\chi_{i4}$ 包含 $x$、$q_d$、$\bar\alpha_{i3}$ 及 $\dot{\bar\alpha}_{i3}$。
未经执行器补偿的名义力矩指令取为
$$
\begin{aligned}
 \nu_i={}&\frac{1}{\hat g_{i4}}
 \bigl[-f_{i4}^{0}(x,t)-k_{i4p}\psi_p(s_{i4})
 -k_{i4q}\psi_q(s_{i4})\\
 &\hspace{9mm}-s_{i3}+\dot{\bar\alpha}_{i3}
 -\hat\vartheta_{i4}\xi_{i4}(\chi_{i4})
 -\rho_{i4}\operatorname{sat}(s_{i4}/\epsilon_{i4})\bigr].
\end{aligned}

 \tag{51}
$$
<a id="eq:nominal_torque"></a>

这里 $\hat g_{i4}$ 可取 $1/J_i$ 的保守估计；当电机惯量存在显著不确定性时，
也可采用投影自适应律更新，使其始终满足 $\hat g_{i4}\geq g_{\min}>0$。

为使每个设计步骤只更新一个标量参数，采用
$$
 \dot{\hat\vartheta}_{ij}=\gamma_{ij}\left[
 s_{ij}\xi_{ij}(\chi_{ij})-\sigma_{ij}\hat\vartheta_{ij}\right],
 \qquad j\in\{2,4\},

 \tag{52}
$$
<a id="eq:adaptation_law"></a>

其中 $\gamma_{ij}>0$、$\sigma_{ij}>0$。与式 (31)一致，泄漏项用于抑制
参数漂移。若在第一个误差面也需逼近参考轨迹中的未知扰动，可增加
$(\hat\vartheta_{i1},\gamma_{i1},\sigma_{i1})$，证明不变。

## 3.3 死区--滞环复合补偿器

<a id="subsec:actuator_compensation"></a>
执行器补偿以 $\nu_i$ 为基准，先进行死区逆补偿。定义连续化符号函数
$$
 \operatorname{sgn}_{\epsilon}(r)=
 \begin{cases}r/|r|,& |r|>\epsilon,\\ r/\epsilon,& |r|\leq\epsilon,\end{cases}

 \tag{53}
$$
<a id="eq:continuous_sign"></a>

并取
$$
 d_{i}^{\mathrm{DZ}}(\nu_i)=
 \hat d_{ri}\,\frac{1+\operatorname{sgn}_{\epsilon_d}(\nu_i)}{2}
 -\hat d_{li}\,\frac{1-\operatorname{sgn}_{\epsilon_d}(\nu_i)}{2}.

 \tag{54}
$$
<a id="eq:dz_comp"></a>

该项把控制指令移出死区；当 $\nu_i>0$ 时增加右侧死区宽度，
当 $\nu_i<0$ 时减小指令以补偿左侧死区。为避免滞环内部状态不可测，
用与式 (8)相同的模型构造状态观测器：
$$
 \dot{\hat z}_i=a_i\dot u_i-b_i|\dot u_i||\hat z_i|^{n_i-1}\hat z_i
 -c_i\dot u_i|\hat z_i|^{n_i},
 \qquad \hat z_i(0)=0,

 \tag{55}
$$
<a id="eq:hysteresis_observer"></a>

其中 $u_i=\mathcal D(v_i)$。滞环补偿项定义为
$$
 h_i^{\mathrm{H}}=-\frac{\hat h_i\hat z_i}{\hat\lambda_i},

 \tag{56}
$$
<a id="eq:h_comp"></a>

其中 $\hat\lambda_i\geq\lambda_{\min}>0$。最终控制指令为
$$
 v_i=\nu_i+d_i^{\mathrm{DZ}}(\nu_i)+h_i^{\mathrm{H}}
 -\frac{\hat r_i}{\hat\lambda_i}
 \operatorname{sat}_{\epsilon_i}(s_{i4}),

 \tag{57}
$$
<a id="eq:control_command"></a>

其中 $\hat r_i$ 是对死区、滞环和外部扰动剩余界的在线估计，
最后一项为连续鲁棒补偿。实际输入力矩满足
$$
 \tau_{a,i}=\lambda_i v_i+\widetilde\varphi_i,
 \qquad
 |\widetilde\varphi_i|\leq\bar\varphi_i
 (1+|v_i|+|z_i-\hat z_i|),

 \tag{58}
$$
<a id="eq:actuator_residual"></a>

因此补偿误差可纳入[本节](#subsec:lyapunov)的有界残差 $\Delta$。为满足
实际执行器幅值约束，可在[式](#eq:control_command) 外增加投影算子
$\operatorname{sat}_{v_{\max}}(\cdot)$；在理论分析中，只要投影误差有界，该项不会改变
预定义时间结论，而只会增大残差集半径。

当死区和滞环界不能离线标定时，$\hat d_{ri}$、$\hat d_{li}$ 和 $\hat h_i$
采用带投影的泄漏更新。例如，对 $b_i\in\{d_{ri},d_{li},h_i\}$ 取
$$
 \dot{\hat b}_i=\operatorname{Proj}_{[0,b_{i,\max}]}
 \left(\gamma_{bi}|s_{i4}|-\sigma_{bi}\hat b_i\right),

 \tag{59}
$$
<a id="eq:actuator_parameter_adaptation"></a>

其中 $b_{i,\max}$ 为由执行器安全工作范围给出的保护上界。若这些参数已由
标定实验获得，则[式](#eq:actuator_parameter_adaptation) 可退化为常值
估计。参数估计误差同样以平方项加入 $V_\vartheta$，其泄漏项贡献的有界
部分并入 $\Delta$。这里 $\operatorname{Proj}_{[0,b_{i,\max}]}(\cdot)$ 表示
到区间 $[0,b_{i,\max}]$ 的欧氏投影。

## 3.4 闭环误差系统

<a id="subsec:closed_loop"></a>
将[式](#eq:alpha_i1)--[式](#eq:control_command)代入系统动力学，可得
每个关节的误差方程
$$
\begin{aligned}
 \dot s_{i1}={}&s_{i2}+y_{i1}-k_{i1p}\psi_p(s_{i1})
                         -k_{i1q}\psi_q(s_{i1}),\\
 \dot s_{i2}={}&-s_{i1}+s_{i3}+y_{i2}-k_{i2p}\psi_p(s_{i2})
                         -k_{i2q}\psi_q(s_{i2})+\omega_{i2},\\
 \dot s_{i3}={}&-s_{i2}+s_{i4}+y_{i3}-k_{i3p}\psi_p(s_{i3})
                         -k_{i3q}\psi_q(s_{i3})+\omega_{i3},\\
 \dot s_{i4}={}&-s_{i3}-k_{i4p}\psi_p(s_{i4})
                         -k_{i4q}\psi_q(s_{i4})+\omega_{i4}.
\end{aligned}

 \tag{60}
$$
<a id="eq:error_dynamics_scalar"></a>

其中 $\omega_{ij}$ 汇总了模型不确定性、滤波器导数、MLP 逼近误差以及死区--
滞环补偿残差。根据假设 1--4 和[式](#eq:actuator_residual)，存在正常数
$\bar\omega_{ij}$ 使得
$$
 |\omega_{ij}|\leq c_{ij,1}|y_{ij}|+c_{ij,2}|\widetilde\vartheta_{ij}|
 +c_{ij,3}|s_{ij}|+\bar\omega_{ij} .

 \tag{61}
$$
<a id="eq:omega_bound"></a>

滤波误差动态由[式](#eq:filter_error_dynamic)给出。令
$$
 S=\operatorname{col}(s_{11},s_{12},s_{13},s_{14},
                       s_{21},s_{22},s_{23},s_{24}),
 \quad
 Y=\operatorname{col}(y_{11},y_{12},y_{13},y_{21},y_{22},y_{23}),

 \tag{62}
$$
<a id="eq:S_Y_definition"></a>

并令 $\widetilde\Theta$ 收集全部标量参数估计误差。[式](#eq:error_dynamics_scalar)
可以写成
$$
 \dot S=-\mathcal K_p\Psi_p(S)-\mathcal K_q\Psi_q(S)
 +\mathcal A S+\mathcal B Y+\Omega(S,Y,\widetilde\Theta,t),

 \tag{63}
$$
<a id="eq:error_dynamics_vector"></a>

其中 $\Psi_r(S)=\operatorname{col}(\psi_r(S_k))$，$\mathcal A$ 为由递推交叉项
组成的有界矩阵，$\mathcal B$ 为滤波误差注入矩阵。

## 3.5 复合 Lyapunov 分析

<a id="subsec:lyapunov"></a>
选取复合 Lyapunov 函数
$$
 V=V_s+V_y+V_\vartheta,

 \tag{64}
$$
<a id="eq:composite_V"></a>

其中
$$
 V_s=\frac12S^{\mathrm T}S,\qquad
 V_y=\sum_{i=1}^{2}\sum_{j=1}^{3}\frac{y_{ij}^{2}}{2\tau_{ij}},qquad
 V_\vartheta=\sum_{i=1}^{2}\sum_{j\in\{2,4\}}
 \frac{\widetilde\vartheta_{ij}^{2}}{2\gamma_{ij}}.

 \tag{65}
$$
<a id="eq:V_components"></a>

若采用自适应死区和滞环界估计，也可在 $V_\vartheta$ 中加入相应参数项。
在将式 (1) 的连杆动力学化为[式](#eq:strict_feedback) 的过程中，
机械能项中的速度耦合满足
$z^{\mathrm T}(\dot M-2C)z=0$。因此，惯性矩阵变化不会产生未被控制的
二次速度项；经过范数等价变换后，其余项均可由[式](#eq:omega_bound)
表示并吸收到递推误差面中。
对[式](#eq:composite_V) 求导，使用式 (5)的斜对称性、适应律
[式](#eq:adaptation_law) 以及 Young 不等式，可分别得到
$$
 s_{ij}\omega_{ij}
 \leq \frac{\eta_{ij}}{2}s_{ij}^{2}
 +\frac{c_{ij,2}^{2}}{2\eta_{ij}}\widetilde\vartheta_{ij}^{2}
 +\frac{c_{ij,1}^{2}}{2\eta_{ij}}y_{ij}^{2}
 +\frac{\bar\omega_{ij}^{2}}{2\eta_{ij}} .

 \tag{66}
$$
<a id="eq:young_omega"></a>

泄漏项产生
$$
 \frac{\widetilde\vartheta_{ij}}{\gamma_{ij}}
 \dot{\hat\vartheta}_{ij}
 \leq -\frac{\sigma_{ij}}{2}\widetilde\vartheta_{ij}^{2}
 +\frac{\sigma_{ij}}{2}(\vartheta_{ij}^{*})^{2}
 +\widetilde\vartheta_{ij}s_{ij}\xi_{ij},

 \tag{67}
$$
<a id="eq:leakage_bound"></a>

其中交叉项 $\widetilde\vartheta_{ij}s_{ij}\xi_{ij}$ 被[式](#eq:adaptation_law)
中相同的 MLP 项抵消。由于
$-y_{ij}^{2}/\tau_{ij}^{2}$ 来自滤波器，选择 $\tau_{ij}$ 足够小并吸收
[式](#eq:young_omega) 中的交叉项，可得
$$
 \dot V\leq -a_p V^{p}-a_q V^{q}-c_y\|Y\|^{2}
 -c_\vartheta\|\widetilde\Theta\|^{2}+\Delta,

 \tag{68}
$$
<a id="eq:Vdot_before_compression"></a>

其中 $a_p,a_q,c_y,c_\vartheta>0$，而
$$
 \Delta=\sum_{i=1}^{2}\sum_{j=1}^{4}
 \frac{\bar\omega_{ij}^{2}}{2\eta_{ij}}
 +\sum_{i=1}^{2}\sum_{j\in\{2,4\}}
 \frac{\sigma_{ij}}{2}(\vartheta_{ij}^{*})^{2}
 +\Delta_{\mathrm{f}}+\Delta_{\mathrm{act}} .

 \tag{69}
$$
<a id="eq:Delta_definition"></a>

这里 $\Delta_{\mathrm f}$ 和 $\Delta_{\mathrm{act}}$ 分别表示滤波器初始误差及
死区--滞环补偿残差造成的有界项。利用有限维范数等价关系，存在
$\alpha>0$、$\beta>0$ 使[式](#eq:Vdot_before_compression)压缩为
$$
 \dot V\leq-\alpha V^{p}-\beta V^{q}+\Delta .

 \tag{70}
$$
<a id="eq:Vdot_final"></a>

参数 $\alpha$、$\beta$ 可由 $k_{ijp}$、$k_{ijq}$ 和滤波器时间常数计算；
实际整定时只需使其满足
$$
 \frac{1}{\alpha(1-p)}+\frac{1}{\beta(q-1)}+\delta\leq T_c,
 \qquad \delta>0 .

 \tag{71}
$$
<a id="eq:Tc_design_condition"></a>

因此 $T_c$ 是控制器设计输入，而不是仿真后由初始状态估计得到的量。

令 $V_\Delta$ 为方程
$$
 \alpha V_\Delta^{p}+\beta V_\Delta^{q}=\Delta

 \tag{72}
$$
<a id="eq:Vdelta_equation"></a>

的最小正根，并定义
$$
 \Omega_V=\{(S,Y,\widetilde\Theta):V\leq V_\Delta\} .

 \tag{73}
$$
<a id="eq:OmegaV"></a>

当 $\Delta=0$ 时 $V_\Delta=0$；当 $\Delta>0$ 时，残差半径可通过减小
滤波时间常数、增加基函数精度和提高鲁棒补偿增益来调节。

## 3.6 预定义时间稳定性定理

<a id="subsec:stability_theorem"></a>
**定理 1（预定义时间实际稳定性）**：在假设 1--4成立、控制律
[式](#eq:alpha_i1)--[式](#eq:control_command)实施且参数满足
[式](#eq:gain_bounds)和[式](#eq:Tc_design_condition)时，闭环系统的所有
信号有界。对于任意初始条件，存在有限时间 $T_{\mathrm{in}}$ 满足
$$
 T_{\mathrm{in}}\leq
 \frac{1}{\alpha(1-p)}+\frac{1}{\beta(q-1)}+\delta
 \leq T_c,

 \tag{74}
$$
<a id="eq:T_in_bound"></a>

使得 $(S,Y,\widetilde\Theta)\in\Omega_V$ 对所有 $t\geq T_{\mathrm{in}}$ 成立。
特别地，若选择 $\varepsilon$ 使
$$
 V_\Delta\leq\frac{\varepsilon^2}{2},

 \tag{75}
$$
<a id="eq:epsilon_condition"></a>

则连杆跟踪误差满足
$$
 \|q(t)-q_d(t)\|\leq\varepsilon,qquad \forall t\geq T_c .

 \tag{76}
$$
<a id="eq:tracking_result"></a>


*证明：* 当 $V>V_\Delta$ 时，由[式](#eq:Vdelta_equation)有
$$
 \dot V\leq-\alpha V^p-\beta V^q<0,

 \tag{77}
$$
<a id="eq:Vdot_outside"></a>

故 $\Omega_V$ 是正不变吸引集。对 $V$ 在 $V\geq1$ 与 $0<V\leq1$ 的两个区域
分别积分，得到
$$
 T_{\mathrm{in}}
 \leq\frac{1}{\beta(q-1)}+\frac{1}{\alpha(1-p)}.

 \tag{78}
$$
<a id="eq:integral_bound"></a>

滤波器瞬态、逼近误差和执行器补偿的实现裕度由 $\delta$ 吸收，于是得到
[式](#eq:T_in_bound)。因为该上界只含控制器参数 $\alpha,\beta,p,q,\delta$
而不含 $V(0)$，所以收敛时间与初始条件无关且可以预先指定。最后，由
[式](#eq:composite_V)有 $\frac12\|q-q_d\|^2\leq V$，结合
[式](#eq:epsilon_condition)即可得到[式](#eq:tracking_result)。证毕。

## 3.7 设计实施说明

<a id="subsec:implementation_notes"></a>
在数值实现中，建议先给定 $T_c$ 和残差精度 $\varepsilon$，再根据
[式](#eq:Tc_design_condition)分配 $\alpha$、$\beta$，最后通过
$k_{ijp}$、$k_{ijq}$ 调整各递推层的瞬态响应。标称仿真取
$T_c=3\,\mathrm{s}$、$p=0.7$、$q=1.3$、$\tau_{ij}=0.01\,\mathrm{s}$、
$\gamma_{ij}=0.5$、$\sigma_{ij}=0.01$。连续饱和函数
$\operatorname{sat}_{\epsilon}(\cdot)$ 应用于实际代码，以避免理想符号函数引起的
高频抖振。对不同 $T_c\in\{1,2,3,4,5\}\,\mathrm{s}$ 重复仿真时，只需
重新计算满足[式](#eq:Tc_design_condition)的双幂次增益，系统模型和
补偿器结构保持不变。这一可调性是预定义时间 DSC 相对于传统渐近 DSC 和
固定时间 DSC 的主要区别。
