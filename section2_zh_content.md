# 2. 系统描述与预备知识

## 2.1 柔性关节机械臂动力学

考虑图1所示的两自由度柔性关节机械臂。该机械臂由刚性连杆子系统和执行器子系统组成，二者通过弹性传动装置连接。令

$$
q=\begin{bmatrix}q_1 & q_2\end{bmatrix}^{\mathrm T}
$$

表示连杆侧关节位置向量，

$$
\theta=\begin{bmatrix}\theta_1 & \theta_2\end{bmatrix}^{\mathrm T}
$$

表示电机侧位置向量。对应的速度向量分别记为 $\dot q$ 和 $\dot\theta$。第 $i$ 个关节的弹性变形定义为

$$
\Delta_i=q_i-\theta_i,\qquad i=1,2.
$$

根据标准柔性关节机械臂模型，连杆侧动力学可表示为

$$
M(q)\ddot q+C(q,\dot q)\dot q+G(q)+F_q\dot q+K(q-\theta)=d_q(t), \tag{1}
$$

其中 $M(q)\in\mathbb{R}^{2\times 2}$ 为正定惯性矩阵，$C(q,\dot q)\in\mathbb{R}^{2\times 2}$ 为科氏力和离心力矩阵，$G(q)\in\mathbb{R}^{2}$ 为重力向量，$F_q\in\mathbb{R}^{2\times 2}$ 为连杆侧黏性摩擦矩阵，

$$
K=\operatorname{diag}\{K_1,K_2\}
$$

为关节刚度矩阵。向量 $d_q(t)$ 表示未知外部干扰和未建模的连杆侧动力学。

电机侧动力学由下式给出：

$$
J\ddot\theta+F_\theta\dot\theta-K(q-\theta)=\tau_a(t)+d_\theta(t), \tag{2}
$$

其中

$$
J=\operatorname{diag}\{J_1,J_2\}
$$

为电机惯量矩阵，$F_\theta=\operatorname{diag}\{F_{\theta 1},F_{\theta 2}\}$ 为电机侧黏性摩擦矩阵，$\tau_a(t)$ 为实际执行器力矩，$d_\theta(t)$ 表示电机侧干扰。

实际执行器力矩由控制指令

$$
v=\begin{bmatrix}v_1 & v_2\end{bmatrix}^{\mathrm T}
$$

经非线性执行器通道产生，即

$$
\tau_a(t)=\mathcal{H}\!\left(\mathcal{D}(v(t))\right), \tag{3}
$$

其中 $\mathcal{D}(\cdot)$ 表示输入死区算子，$\mathcal{H}(\cdot)$ 表示滞环算子。

对于仿真研究所考虑的两连杆平面机械臂，标称物理参数选取为

$$
m_1=1.0~\mathrm{kg},\qquad m_2=1.5~\mathrm{kg},
$$

$$
l_1=1.0~\mathrm{m},\qquad l_2=0.8~\mathrm{m},
$$

以及

$$
l_{c1}=0.5~\mathrm{m},\qquad l_{c2}=0.4~\mathrm{m}.
$$

连杆惯量参数记为 $I_1$ 和 $I_2$，$g$ 表示重力加速度。标称电机惯量、关节刚度和黏性摩擦系数选取为

$$
J_1=J_2=0.02~\mathrm{kg\,m^2},
$$

$$
K_1=K_2=100~\mathrm{N\,m/rad},
$$

以及

$$
F_{\theta 1}=F_{\theta 2}=0.5~\mathrm{N\,m\,s/rad}.
$$

式 (1)–(2) 描述了每个柔性关节的四阶动力学。因此，完整的两关节机械臂具有八个机械状态。定义

$$
x=\begin{bmatrix}q^{\mathrm T} & \dot q^{\mathrm T} & \theta^{\mathrm T} & \dot\theta^{\mathrm T}\end{bmatrix}^{\mathrm T},
$$

系统状态维数为八。连杆侧惯性矩阵满足

$$
0<\underline m I_2\leq M(q)\leq \overline m I_2, \tag{4}
$$

其中 $\underline m$ 和 $\overline m$ 为未知正常数。此外，假设标准斜对称性成立：

$$
\dot M(q)-2C(q,\dot q) \quad \text{是斜对称的}. \tag{5}
$$

因此，对于任意向量 $z\in\mathbb{R}^{2}$，

$$
z^{\mathrm T}\left(\dot M(q)-2C(q,\dot q)\right)z=0.
$$

该性质对于在 Lyapunov 分析中消除速度相关项至关重要。

<callout emoji="🖼️"><p><b>图1</b>：两自由度柔性关节机械臂示意图。（待插入）</p></callout>

## 2.2 执行器死区与滞环

在实际机器人系统中，执行器力矩通常与指令控制输入并不相同。具体而言，间隙、静摩擦、放大器限制和传动不完善均会产生死区效应。采用非对称死区模型如下：

$$
\mathcal{D}(v_i)=\begin{cases}v_i-d_{ri}, & v_i>d_{ri},\\[2mm]0, & -d_{li}\leq v_i\leq d_{ri},\\[2mm]v_i+d_{li}, & v_i<-d_{li},\end{cases} \tag{6}
$$

其中 $d_{ri}>0$ 和 $d_{li}>0$ 分别为第 $i$ 个执行器的右侧和左侧死区宽度。死区参数不一定对称，因为正向和反向驱动路径可能具有不同的摩擦和传动特性。

滞环效应采用简化的 Bouc–Wen 型内部状态模型描述：

$$
\tau_{a,i}=k_{a,i}u_i+h_i z_i, \tag{7}
$$

其中

$$
u_i=\mathcal{D}(v_i)
$$

为经死区滤波的指令，$k_{a,i}>0$ 为标称执行器增益，$h_i$ 为滞环幅值，$z_i$ 为内部滞环状态。$z_i$ 的演化由下式描述：

$$
\dot z_i=a_i\dot u_i-b_i|\dot u_i||z_i|^{n_i-1}z_i-c_i\dot u_i|z_i|^{n_i}, \tag{8}
$$

其中 $a_i$、$b_i$、$c_i$ 为正模型参数，$n_i\geq 1$ 决定滞环回线的形状。

在控制器设计中，无需精确知道执行器参数。只需假设执行器非线性满足有界分解：

$$
\tau_{a,i}=\lambda_i v_i+\varphi_i(v_i,z_i), \tag{9}
$$

其中 $\lambda_i$ 为未知正控制增益，$\varphi_i(v_i,z_i)$ 为有界非线性残差，满足

$$
|\varphi_i(v_i,z_i)|\leq\bar\varphi_i\left(1+|v_i|+|z_i|\right), \tag{10}
$$

$\bar\varphi_i$ 为未知正常数。该表示将主控制通道与非线性执行器不确定性分离，使得死区和滞环效应可在自适应控制框架内进行补偿。

死区和滞环参数假设满足

$$
0<\underline d_{ri}\leq d_{ri}\leq\overline d_{ri},\qquad 0<\underline d_{li}\leq d_{li}\leq\overline d_{li}, \tag{11}
$$

以及

$$
|h_i|\leq\overline h_i,\qquad |z_i(t)|\leq\overline z_i, \tag{12}
$$

其中所有界均为有限值但可能未知。这些条件对于在正常温度、速度和负载范围内工作的机电执行器而言是物理合理的。

## 2.3 控制目标

令

$$
q_d(t)=\begin{bmatrix}q_{d1}(t) & q_{d2}(t)\end{bmatrix}^{\mathrm T}
$$

为期望的连杆侧轨迹。跟踪误差定义为

$$
e(t)=q(t)-q_d(t). \tag{13}
$$

控制目标是设计一个动态面控制器，使以下要求同时满足：

1. 连杆侧位置 $q_i(t)$ 以任意小的残差跟踪期望轨迹 $q_{di}(t)$；
2. 所有闭环信号——包括连杆侧位置、电机侧位置、自适应参数、滤波器状态和执行器指令——保持有界；
3. 跟踪误差在与初始条件无关的用户选定预定义时间 $T_c>0$ 内进入预设残差集。

对于给定的精度水平 $\varepsilon>0$，定义残差跟踪集

$$
\Omega_\varepsilon=\left\{e\in\mathbb{R}^{2}:\|e\|\leq\varepsilon\right\}. \tag{14}
$$

因此，期望的预定义时间跟踪特性表示为

$$
e(t)\in\Omega_\varepsilon,\qquad \forall t\geq T_c. \tag{15}
$$

$T_c$ 的值由设计者选定，而非在控制器参数确定后获得。在仿真研究中，标称预定义收敛时间选取为

$$
T_c=3~\mathrm{s}.
$$

实际目标并非迫使物理状态在有限时间内精确为零——这通常与有界执行器动力学和测量噪声不相容——而是保证收敛到一个可调邻域，其大小可通过控制器增益和逼近精度加以减小。

## 2.4 假设条件

在后续控制器设计和稳定性分析中，施加以下假设。

<callout emoji="📐" background-color="light-blue"><p><b>假设1</b>：期望轨迹 $q_d(t)$ 二阶连续可微，且存在未知正常数 $\bar q_d$、$\bar{\dot q}_d$、$\bar{\ddot q}_d$，使得</p>

<p>$$
\|q_d(t)\|\leq\bar q_d,\qquad \|\dot q_d(t)\|\leq\bar{\dot q}_d,\qquad \|\ddot q_d(t)\|\leq\bar{\ddot q}_d. \tag{16}
$$</p>

<p>该假设由标准工业轨迹规划器满足，后者通常生成具有连续速度和加速度的位置参考。它还防止虚拟控制信号中出现不连续性，避免产生无界的执行器指令。</p></callout>

<callout emoji="📐" background-color="light-blue"><p><b>假设2</b>：死区和滞环参数满足式 (11)–(12) 中的有界性条件。执行器非线性关于其自变量连续，且具有有界内部状态。</p>

<p>该假设反映了执行器非线性通常在规定工作范围内有界的事实。模型参数无需精确已知，因为其不确定性通过自适应补偿和鲁棒残差项来处理。</p></callout>

<callout emoji="📐" background-color="light-blue"><p><b>假设3</b>：关节刚度矩阵正定且有界：</p>

<p>$$
0<\underline K I_2\leq K\leq\overline K I_2, \tag{17}
$$</p>

<p>其中 $\underline K$ 和 $\overline K$ 为未知正常数。</p>

<p>该假设排除了弹性的完全丧失，保证弹性变形在物理上有意义。对于扭转刚度保持在已知机械范围内的柔性传动而言，该假设是有效的。</p></callout>

<callout emoji="📐" background-color="light-blue"><p><b>假设4</b>：集总不确定性和干扰项满足匹配增长条件。具体而言，对于每个关节，存在未知正常数 $\bar d_i$，使得</p>

<p>$$
|d_i(t)|\leq\bar d_i\left(1+\|x(t)\|+\|q_d(t)\|\right), \tag{18}
$$</p>

<p>其中 $d_i(t)$ 表示与第 $i$ 个递推控制步骤相关的集总不确定性。</p>

<p>该假设意味着主导不确定性通过控制输入相同的通道进入，或者可在标准反馈变换后有上界。外部负载干扰、未建模摩擦和刚性连杆动力学中的参数变化均可由此集总项表示。</p></callout>

## 2.5 理论预备

### 2.5.1 预定义时间稳定性

考虑非线性系统

$$
\dot x=f(x,t),\qquad x(0)=x_0. \tag{19}
$$

**定义1**：若对于用户选定的常数 $T_c>0$，系统 (19) 的每个解都满足

$$
\lim_{t\rightarrow T_c}x(t)=0, \tag{20}
$$

且收敛时间上界由 $T_c$ 显式确定，与初始条件 $x_0$ 无关，则称平衡点 $x=0$ 是预定义时间稳定的。

对于实际跟踪，精确收敛被收敛到残差集所替代。一个有用的 Lyapunov 条件如下。

**引理1**：设连续可微的正定函数 $V(x)$ 满足

$$
\dot V\leq-\alpha V^{p}-\beta V^{q}+\Delta, \tag{21}
$$

其中

$$
\alpha>0,\qquad \beta>0,\qquad 0<p<1,\qquad q>1,\qquad \Delta\geq 0.
$$

则系统是一致最终有界的，且所有轨迹在由 $T_c$ 确定的预定义时间内进入残差集

$$
\Omega_V=\left\{x:V(x)\leq V_\Delta\right\}, \tag{22}
$$

其中 $V_\Delta$ 为满足下式的最小正解：

$$
\alpha V_\Delta^{p}+\beta V_\Delta^{q}=\Delta. \tag{23}
$$

在理想情形 $\Delta=0$ 时，预定义时间收敛界可表示为

$$
T_{\mathrm{conv}}\leq\frac{1}{\alpha(1-p)}+\frac{1}{\beta(q-1)}+\delta, \tag{24}
$$

其中 $\delta>0$ 为设计裕度，用于考虑滤波器动力学、逼近误差和执行器非线性。参数 $\alpha$、$\beta$、$p$、$q$ 的选取使得

$$
\frac{1}{\alpha(1-p)}+\frac{1}{\beta(q-1)}+\delta\leq T_c. \tag{25}
$$

在所提设计中，指数为 $p$ 和 $q$ 的非线性阻尼项分别用于调节小误差和大误差区域的收敛速率。当跟踪误差较小时 $V^p$ 项占主导，而当初始误差较大时 $V^q$ 项加速衰减。因此，收敛时间界可独立于系统初始状态进行设定。

### 2.5.2 动态面滤波

动态面控制方法引入一阶滤波器，以避免对虚拟控制信号的重复解析微分。对于虚拟控制信号 $\alpha_j(t)$，定义其滤波版本 $\bar\alpha_j(t)$ 为

$$
\tau_j\dot{\bar\alpha}_j+\bar\alpha_j=\alpha_j,\qquad \bar\alpha_j(0)=\alpha_j(0), \tag{26}
$$

其中 $\tau_j>0$ 为滤波器时间常数。滤波误差定义为

$$
y_j=\bar\alpha_j-\alpha_j. \tag{27}
$$

对式 (27) 求导并利用式 (26)，可得

$$
\dot y_j=-\frac{1}{\tau_j}y_j-\dot\alpha_j. \tag{28}
$$

滤波器时间常数决定了逼近精度与瞬态控制 effort 之间的折中。较小的 $\tau_j$ 提供对未滤波虚拟控制更接近的逼近，但可能放大高频分量。本研究中，标称值选取为

$$
\tau_j=0.01~\mathrm{s}.
$$

由于期望轨迹连续可微，且所提设计下所有虚拟控制信号有界，滤波误差保持有界。

### 2.5.3 最小参数学习

为降低在线计算负担，采用最小参数学习结构来逼近未知非线性函数。令 $\varphi_j(\chi_j)$ 表示回归向量 $\chi_j$ 的未知连续非线性函数，其逼近为

$$
\varphi_j(\chi_j)=\vartheta_j^{*}\xi_j(\chi_j)+\epsilon_j(\chi_j), \tag{29}
$$

其中 $\xi_j(\chi_j)$ 为已知基函数，$\vartheta_j^{*}$ 为未知理想标量参数，$\epsilon_j(\chi_j)$ 为有界逼近误差，满足

$$
|\epsilon_j(\chi_j)|\leq\bar\epsilon_j. \tag{30}
$$

自适应估计 $\hat\vartheta_j$ 按下式更新：

$$
\dot{\hat\vartheta}_j=\gamma_j\left(s_j\xi_j-\sigma_j\hat\vartheta_j\right), \tag{31}
$$

其中 $s_j$ 为对应的递推误差面，$\gamma_j>0$ 为自适应增益，$\sigma_j>0$ 为泄漏系数。泄漏项可在存在干扰和逼近误差时防止参数漂移。

定义参数估计误差为

$$
\tilde\vartheta_j=\hat\vartheta_j-\vartheta_j^{*}. \tag{32}
$$

利用不等式

$$
ab\leq\frac{\rho}{2}a^2+\frac{1}{2\rho}b^2,\qquad \rho>0, \tag{33}
$$

与逼近误差相关的交叉项可被约束为

$$
|s_j\epsilon_j|\leq\frac{\eta_j}{2}s_j^2+\frac{1}{2\eta_j}\bar\epsilon_j^2, \tag{34}
$$

其中 $\eta_j>0$ 为任意设计常数。因此，逼近误差仅对闭环 Lyapunov 导数贡献一个有界残差项。

数值实现中，自适应增益和泄漏系数选取为

$$
\gamma_j=0.5,\qquad \sigma_j=0.01.
$$

每个递推设计步骤仅更新一个标量自适应参数。因此，所提最小参数学习结构避免了传统神经网络权值自适应带来的大计算量，同时保留了补偿未知非线性效应的能力。

式 (21)、(33)、(34) 中的不等式以及式 (5) 的斜对称性，将在第3章中用于构造复合 Lyapunov 函数，并建立柔性关节机械臂的预定义时间实际稳定性。
