---
tags:
  - 卡尔曼滤波
  - 拓展卡尔曼滤波
  - python
---
- 培训时间：/
- 讲述人：虞若弘
- 培训内容：卡尔曼滤波的原理与推导
- 培训目标：大致了解卡尔曼滤波并尝试手写
# 写在前面
### 一个有趣小故事：
           https://zhuanlan.zhihu.com/p/93011093

### 卡尔曼滤波

将预测状态量的高斯分布和观测量的高斯分布做融合，生成一个新的高斯分布，其中新的高斯分布的均值和方差是两个独立的高斯分布的相关参量的加权，这个加权就是卡尔曼增益，但是预测状态量和观测量可能维度不同，需要将他们同时转换到一个向量空间中，所以观测量前面有线性变换矩阵。
# 一、递归算法

### 本质
最优化递归数据处理算法（Optimal Recursion Data Processing  Algorithm）。
### 计算方法

$$\begin{array}{rl}\hat{x}_{k}&=\frac1{k}\left(z_1+z_1+\cdots+z_k\right)\\&=\frac1{kk}\left(z_1+z_2+\cdots+z_{k-1}\right)+\frac1kz_k\\&=\frac1{k}\frac{k-1}{(k-1)}\left(z_1+z_1+z_1+\cdots+z_{k-1}\right)+\frac1kz_k\\&=\frac{k-1}{k}\chi_{k-1}^n+\frac1{k}z_k\\&=\hat{\chi}_{k-1}-\frac1{k}\hat{\chi}_{k-1}+\frac1{k}z_k\end{array}$$
其中 $Z_i,\quad i =1, 2, 3, ……, k$  表示第 i 次的状态测量结果且 $\overrightarrow{Z_{i}}=(\vec{p},\vec{\nu})$。$\vec{p}$ 为位置向量，$\vec{v}$ 为速度向量。
##### Tips：
随着测量结果 k 的增加，测量结果不再重要。相反，k很小的时候，$\frac{1}{k}$ 较大，$Z_k$ 影响较大。

### 公式描述

$$\text{当前的估计值 = 上一次的估计值 + 系教 ×(当前测量值 - 上一次的估计值)}$$

### 一些概念

1. 估计误差： 当前估计值与上一次估计值之差，用符号 $e_{EST}$ 表示。
2. 测量误差： 当前测量值与当前估计值之差，用符号 $e_{MEA}$ 表示。
3. 卡尔曼增益（Kalmen Gain），即上式中的系数，用符号 $k_k$ 表示。
### 核心公式
$$k_k=\frac{e_{{ssT}_{k-1}}}{e_{{BT}_{k-1}}+e_{{M64}_k}}$$
#### Tips:
在 k 时刻，
	1. 若 $e_{EST_{k-1}}\gg e_{MBA_{k}}$，$k_k$ 则会趋近于1。可得 $\hat{x}_{k}=\hat{x}_{k-1}+z_{k}-\hat{x}_{k-1}=z_{k}$ 克服了平均响应慢的问题。即，测量误差很小时，估计值等于测量值。
	2. 若 $e_{EST_{k-1}}\ll e_{MEA_k}$ ，$k_k$ 则会趋近于0。可得 $\hat{x_{k}}=\hat{x}_{k-1}$ 。即，测量误差很大时，选择相信上一次的估计值。

### 举例
$\text{x=50mm}\quad\hat{x}_{0}=40mm\quad e_{tST_{0}}=5mm\quad z_{1}=51mm\quad e_{MEA_{k}}=3mm$

|    k     |   $Z_k$  |  $e_{{MEA}_{k}}$  |  $\hat{x_k}$   |   $k_k$   |  $e_{EST_k}$  |
|    :-:   |    :-:  |   :-:      |    :-:      |      :-:       |        :-:  |
| 0  |     |    |  40  |        |    5   |
| 1 | 51 |  3  | 46.875 | 0.625 | 1.875 |
|2| 48 | 3 | 47.308 | 0.3846 | 1.154 |
|3| 47 | 3 | 47.220 | 0.278 | 0.833 |

**当 k = 1 时：**
$$k_{k}=\frac{5}{5+3}=0.625$$
$$x_{k}=40+0.625 * (51-40)=46.875$$
$$e_{{EST}}=(1-0.625) * 5=1.875$$
**当 k = 2 时：**
$$k_{k}=\frac{1.875}{1.875 + 3}=0.3846$$
$$\hat{x}_{k}=46.875+0.3846 * (48-46.875)=47.308$$
$$e_{EST}=(1-0.3841) * 1.875=1.154$$
![[误差估计实例.png]]



# 二、数据融合、协方差矩阵、状态空间矩阵
### 数据融合（Data Fusion）
如下过程即为数据融合（通过融合多个传感器数据实现比单传感器更精准判断）。

![[数据融合1.png]]

![[数据融合2.png]]

#### Tips:
   1. 现有两个观测值 $Z_1$ $Z_2$ 与其对应的标准差且**服从正态分布**，通过此来预测 $\hat{Z}$。
   2. 大致在两图像中间，采用公式 $\hat{Z} = Z_1 + k * (Z_2 - Z_1)$ 进行数学推导，求出最小方差。

### 协方差矩阵（covariance matrix）

![[协方差矩阵.png]]

### 状态空间矩阵（State space system matrix）
### 定义
顾名思义，描述该系统状态空间的矩阵。
能够完全表征系统运动状态的**最小**一组变量（用这组变量，能够完全确定系统在任意时刻的状态）

数学解释：**他们是系统所有内部变量中线性无关的一个极大变量组，除选取的变量以外，其他的变量都与他们线性相关**。

![[小车系统.png]]

由牛顿第二定律 $M\frac{d^2y}{dt^2}=u-k\frac{dy}{dt}$ 得一阶微分方程组
$$\begin{cases}\dfrac{dx_1}{dt}=\dfrac{dy}{dt}=x_2\\\dfrac{dx_2}{dt}=\dfrac{d^2y}{dt^2}=-\dfrac{k}{M}\dfrac{dy}{dt}+\dfrac{1}{M}u=-\dfrac{k}{M}x_2+\dfrac{1}{M}u\end{cases}$$
转化成状态方程：
$$\begin{bmatrix}\dot{x_1}\\\dot{x_2}\end{bmatrix}=\begin{bmatrix}0&1\\0&-\frac kM\end{bmatrix}\begin{bmatrix}x_1\\x_2\end{bmatrix}+\begin{bmatrix}0\\\frac1M\end{bmatrix}u$$
可简记为下式：
$$\dot{x}=Ax+bu$$
状态空间方程表示为：
$$\begin{array}{rcl}{{x_{k}=}}&{{AX_{k-1}+B u_{k-1}+W_{k-1}}}\\{{Z_{k}=}}&{{HX_{k}+V_{k}}}\\\end{array}$$
其中 $X_k$ 为状态变量，A 为状态矩阵， B 为控制矩阵，$u_k$ 为控制，$W_{k-1}$ 为过程噪声，$V_K$ 为测量噪声

# 三、卡尔曼增益数学推导

假设一个确定性离散时间系统用以下**状态空间方程（实际上去掉了噪音）**：
$$\begin{aligned}x_{k+1}&=Ax_k+Bu_k\\\\y_k&=Cx_k\end{aligned}$$
则，对于状态的估计为：
$$\hat{x}_k=\hat{x}_k^{cal}+G(\hat{x}_k^{mes}-\hat{x}_k^{cal})=\hat{x}_k^{cal}+K_k(y_k-C\hat{x}_k^{cal})$$
我们现在希望估计值 $\hat x_k$ 和真实值 $x_k$ **越接近越好**，故来考察二者之间的差距：
$$\begin{aligned}
e_k= 
&x_k-\hat{x}_k=x_k-(\hat{x}_k^{cal}+K_k(y_k-C\hat{x}_k^{cal})) \\
&=x_k-\hat{x}_k^{cal}-K_k(Cx_k+v_k)+K_kC\hat{x}_k^{cal} \\
&=(I-K_kC)x_k-(I-K_kC)\hat{x}_k^{cal}-K_kv_k \\
&=(I-K_kC)(x_k-\hat{x}_k^{cal})-K_kv_k
\end{aligned}$$
进一步地，我们假设这个误差也是**正态分布**：
$$e_k\sim{N}(0,P_k),P_k=E[e_ke_k^T]$$
那么就是希望误差满足的这个正态分布的方差越小越好，即转化成了误差的协方差**矩阵的迹**最小，因此需要推导协方差矩阵表达式。
$$ P_k = E[e_ke_k^T]$$
代入 $e_k$ 得：
$$P_k=E\{[(I-K_kC)(x_k-\hat{x}_k^{cal})-K_kv_k][(I-K_kC)(x_k-\hat{x}_k^{cal})-K_kv_k]^T\}$$
计算矩阵的转置：
$$P_k=E\{[(I-K_kC)(x_k-\hat{x}_k^{cal})-K_kv_k][(x_k-\hat{x}_k^{cal})^T(I-K_kC)^T-v_k^TK_k^T]\}$$
引入 $\hat{e_k}$ ：
$$\begin{aligned}&P_k=E[(I-K_kC)\hat{e}_k\hat{e}_k^T(I-K_kC)^T-(I-K_kC)\hat{e}_kv_k^TK_k^T\\&-K_kv_k\hat{e}_k^T(1-K_kC)^T+K_kv_kv_k^TK_k]\end{aligned}$$
因为是线性加减，故而可以对每一项进行求解期望
$$\begin{aligned}E[(I-K_kC)\hat{e}_kv_k^TK_k^T]&=(I-K_kC)E[\hat{e}_kv_k^T]K_k^T=(I-K_kC)E[\hat{e}_k]E[v_k^T]K_k^T\\&=0\end{aligned}$$
$$E[K_kv_k\hat{e}_k^T(1-K_kC)^T]=0$$
代回原式可得：
$$P_k=(I-K_kC)E[\hat{e}_k\hat{e}_k^T](I-K_kC)^T+K_kE[v_kv_k^T]K_k^T$$

将误差的协方差矩阵记作 $\hat{P}_{k}=E[\hat{e}_{k}\hat{e}_{k}^{T}]$ 。

由此，推导出协方差矩阵的表达式：
$$P_k=\hat{P}_k-K_kC\hat{P}_k-\hat{P}_kC^TK_k^T+K_kC\hat{P}_kC^TK_k^T+K_kRK_k^T$$
现要求协方差矩阵的迹最小，故直接求导。
首先，引入下列两个公式
$$\begin{aligned}\frac{d tr(AB)}{dA}&=B^T\\\frac{dtr(ABA^T)}{dA}&=2AB\end{aligned}$$
那么，开始计算
$$\begin{aligned}
&\frac{dtr(P_k)}{dK_k}=0-2\quad(C\hat{P}_k)^T+2K_kC\hat{P}_kC^T+2K_kR=0 \\
&-\hat{P}_kC^T+K_k(C\hat{P}_kC^T+R)=0 \\
&K_{k}=\frac{\hat{P}_{k}C^{T}}{C\hat{P}_{k}C^{T}+R}
\end{aligned}$$

由此，推导出卡尔曼增益的公式，撒花！
# 四、误差协方差矩阵数学推导

先总结一下我们的假设，
$$\begin{array}{ll}{{x_{k}=Ax_{k-1}+BU_{k-1}+W_{k-1}\quad}}&{{w\sim p(0,Q)}}\\{{z_{k}=Hx_{k}+\nu_{k}}}&{{V\sim p(0,R)}}\\\end{array}$$
以及已经推导出的公式
1. 先验估计
$$\hat{x_{k}^{-}}=A\hat{x}_{k-1}+Bu_{k-1}$$
2. 后验估计（数据融合）
$$\hat{x}_{k}=\hat{x}_{k}^{-}+k_{k}(z_{k}-H\hat{x}_{k}^{-}).$$
3. 卡尔曼增益
$$k_{k}=\frac{P_{k}^{-}H^{T}}{HP_{k}^{-}H^{T}+R}$$
下面来求解 $P_{k}^{-} = E [e_k^-e_k^{-T}]$ 
$$\begin{array}{c}{{e_{k}^{-}=x_{k}^{-} - \hat{x}_{k}}}
\\{{=Ax_{k-1}+Bu_{k-1}+w_{k-1}}}{{-A\hat{x}_{k-1}-Bu_{k-1}}}
\\=A(x_{k-1} - \hat{x}_{k-1}) + w_{k-1}
\\=Ae_{k-1} + w_{k-1}
\end{array}$$
代入 $P_k^-$ 得
$${P_{k}^{-} = E [e_k^-e_k^{-T}]}
\\=AE[e_{k-1}e_{k-1}^T]A^T + E[w_{k-1}w_{k-1}^T]
\\=AP_{k-1}A^T + Q$$
# 五、卡尔曼滤波器公式总结

$$\begin{array}{ll}{{\chi_{k}=A\chi_{k-1}+BU_{k-1}+W_{k-1}\quad}}&{{w\sim p(0,Q)}}\\{{z_{k}=H\chi_{k}+\nu_{k}}}&{{V\sim p(0,R)}}\\\end{array}$$

| | 预测 | 校正 |
| :-: | :-: | :-: |
| 先验 | $\hat{x_{k}^{-}}=A\hat{x}_{k-1}+Bu_{k-1}$ | 卡尔曼增益 $k_{k}=\frac{P_{k}^{-}H^{T}}{HP_{k}^{-}H^{T}+R}$ |
| 先验误差协方差 | $P_k^-=AP_{k-1}A^T + Q$ | 后验估计 $\hat{\chi}_{k}=\hat{\chi}_{k}^{-}+k_{k}(z_{k}-H\hat{x}_{k}^{-})$ |
| | | 更新误差协方差 $P_k = (I - k_kH)P_k^-$ |
# 六、运行实例

链接： https://pan.baidu.com/s/1GdJe2eWIlaQrk2nrjemRCQ
提取码：txn3
密码： 6.66
密码推导过程： 6.5 + $\frac{0.2^2}{0.2^2 + 0.4^2}$ * (7.3 - 6.5) = 6.66

# 七、拓展卡尔曼滤波（EKF）

现在考虑非线性系统：
$$
\begin{aligned}
{x_k = f(x_{k-1}, u_{k-1}, w_{k-1}), \qquad P(w)\sim N(0, Q)}
\\{z_k = h(x_k, v_k), \qquad P(v)\sim N(0, R)}
\end{aligned}
$$
这会导致正态分布的随机变量通过非线性系统之后就不再是正态分布的了。
故而选择 $f(x_k)$ 在 $\hat{x_{k-1}}$ 处线性化，可得：
$$\begin{array}{c}{x_k = f(\hat{x_{k-1}}, u_{k-1}, w_{k-1}) + A(x_k - \hat{x_{k-1}}) + Ww_{k-1}}
\\{A=\frac{\partial f}{\partial x}\mid\hat{x}_{k-1},u_{k-1}}
\\{W=\frac{\partial f}{\partial x}\mid\hat{x}_{k-1},u_{k-1}}
\end{array}$$
记 $\widetilde{x_k} = f(\hat{x_{k-1}}, u_{k-1}, 0)$ ，
选择 $z_k$ 在 $\widetilde{x}$ 处线性化
$$\begin{array}{c}{z_k = h(\widetilde{x}, v_k) + H(x_k - \widetilde{x}) + Vv_k}
\\{{H=\frac{\partial h}{\partial x}\mid\tilde{x}_{k}}}\\{{V=\frac{\partial h}{\partial\nu}\mid\tilde{x}_{k}}}
\end{array}$$

| | 预测 | 校正 |
| :-: | :-: | :-: |
| 先验 | $x_k = f(x_{k-1}, u_{k-1}, 0)$ | 卡尔曼增益 $k_{k}=\frac{P_{k}^{-}H^{T}}{HP_{k}^{-}H^{T}+VRV^T}$ |
| 先验误差协方差 | $P_k^-=AP_{k-1}A^T + WQW^T$ | 后验估计 $\hat{x}_{k}=\hat{x}_{k}^{-}+k_{k}(z_{k}-h(\hat{x}_{k}^{-},0))$ |
| | | 更新误差协方差 $P_k = (I - k_kH)P_k^-$ |

# 八、数学知识

### 协方差

#### 定义
刻画两个随机变量 X、Y 之间的相关性。

如果两个变量的变化趋势一致，也就是说如果其中一个大于自身的期望值，另外一个也大于自身的期望值，那么两个变量之间的协方差就是正值。

如果两个变量的变化趋势相反，即其中一个大于自身的期望值，另外一个却小于自身的期望值，那么两个变量之间的协方差就是负值。

**方差**就是协方差的一种**特殊形式**，当两个变量相同时，协方差就是方差了。
#### 公式
$$\sigma(x,y)=\frac1{n-1}\sum_a^b(x_i-\bar{x})(y_i-\bar{y})$$
### 协方差矩阵
#### 公式
$$\Sigma=\begin{bmatrix}\sigma(x_1,x_1)&\cdots&\sigma\left(x_1,x_d\right)\\\vdots&\ddots&\vdots\\\sigma\left(x_d,x_1\right)&\cdots&\sigma(x_d,x_d)\end{bmatrix}\in\mathbb{R}^{d\times d}$$
#### 特征值分解
$$\Sigma=U\Lambda U^T$$
其中，$U$ 的每一列都是相互正交的特征向量，且是单位向量，满足 $U^TU=I$  $\Lambda$ 对角线上的元素是从大到小排列的特征值，非对角线上的元素均为0。

由此，可化成下式：
$$\Sigma=\left(U\Lambda^{1/2}\right)\left(U\Lambda^{1/2}\right)^T=AA^T$$
因此，通俗地说，**任意一个协方差矩阵都可以视为线性变换的结果**。

#### python 代码实现

```python
import numpy as np
import matplotlib.pyplot as plt

#计算协方差
def cov(x1,x2):
    x1mean, x2mean = x1.mean(), x2.mean()
    Sigma = np.sum((x1-x1mean) * (x2-x2mean)) / (len(x1)-1)
    return Sigma

#协方差矩阵 
def covMatrix(X):
    matrix = np.array([[cov(X[0],X[0]), cov(X[0], X[1])],                                                      [cov(X[1],X[0]), cov(X[1],X[1])]])
    return matrix
```

### 矩阵的迹
#### 定义

一个 n 阶方阵中主对角线上各个元素的总和，记作 tr( )
#### 性质

矩阵 A 的迹 = 矩阵 A 的所有特征值之和
# 九、 卡尔曼滤波 python 实现

**预测公式**

$$\begin{aligned}&\mathbf{\hat{x}}_k=\mathbf{F}_k\mathbf{\hat{x}}_{k-1}+\mathbf{B}_k\mathbf{\vec{u}}_k\\\\&\mathbf{P}_k=\mathbf{F}_k\mathbf{P}_{k-1}\mathbf{F}_k^T+Q_k\end{aligned}$$

  

**分布转换公式**

$$\begin{aligned}\vec{\mu}_{\mathrm{expected}}&=\mathbf{H}_k\mathbf{\hat{x}}_k\\\mathbf{\Sigma}_{\mathrm{expected}}&=\mathbf{H}_k\mathbf{P}_k\mathbf{H}_k^T\end{aligned}$$

  

**更新阶段公式**

$$\begin{aligned}
&\text{K}^{\prime}={\mathbf{P}_k\mathbf{H}_k^T(\mathbf{H}_k\mathbf{P}_k\mathbf{H}_k^T+R_k)^{-1}}  \\
&\hat{\mathbf{x}}_k^{\prime}={\mathbf{\hat{x}}}_k+\mathbf{K}^{\prime}(\overset{\rightharpoonup}{\operatorname*{z}}_k{-}{\mathbf{H}}_k\hat{\mathbf{x}}_k)  \\
&\mathbf{P}_k^{\prime}={\mathbf{P}}_k–\mathbf{K}^{\prime}{\mathbf{H}}_k{\mathbf{P}}_k
\end{aligned}$$

```python
class KalmanFilter(object):

    def __init__(self) -> None:
        ndim, dt = 4, 1
        # 创建模型矩阵
        self._motion_mat = np.eye(2 * ndim, 2 * ndim)
        for i in range(ndim):
            self._motion_mat[i, ndim + i] = dt
        self._update_mat = np.eye(ndim, 2 * ndim)
        self._std_weight_position = 1. / 20
        self._std_weight_velocity = 1. / 160

  
    def initiate(self, measurement):
        '''
        函数说明: 初始化状态与状态协方差
        Parameters:
            measurement : ndarray - (x, y, a, h), 中心位置 (x, y), 长宽比 a, 高度 h
        Returns:
            (ndarray, ndarray) - 返回新轨迹的均值向量 (8维)和协方差矩阵(8*8维) 未观测到的速度初始化均值为0
        '''
        mean_pos = measurement
        mean_vel = np.zeros_like(mean_pos)
        mean = np.r_[mean_pos, mean_vel]
        std = [2 * self._std_weight_position * measurement[3],
               2 * self._std_weight_position * measurement[3],
               1e-2,
               2 * self._std_weight_position * measurement[3],
               10 * self._std_weight_velocity * measurement[3],
               10 * self._std_weight_velocity * measurement[3],
               1e-5,
               10 * self._std_weight_velocity * measurement[3]]
        covariance = np.diag(np.square(std))
        
        return mean, covariance


    def predict(self, mean, covariance):
        '''
        函数说明: 预测
        Parameters:
            mean : ndarray - 上一时间的8维物体平均状态向量
        Returns:
            (ndarray, ndarray) - 返回预测阶段的均值向量和协方差矩阵
        '''
        std_pos = [self._std_weight_position * mean[3],
                   self._std_weight_position * mean[3],
                   1e-2,
                   self._std_weight_position * mean[3]]
        std_vel = [self._std_weight_velocity * mean[3],
                   self._std_weight_velocity * mean[3],
                   1e-5,
                   self._std_weight_velocity * mean[3]]
        motion_cov = np.diag(np.square(np.r_[std_pos, std_vel]))
        mean = np.dot(self._motion_mat, mean)
        covariance = np.linalg.multi_dot((self._motion_mat, covariance, self._motion_mat.T)) + motion_cov

        return mean, covariance
  

    def project(self, mean, covariance):
        '''
        函数说明: 分布转换
        Parameters:
            mean : ndarray - 上一时间的8维物体平均状态向量
            covariance : ndarray - 协方差矩阵(8*8维)
        Returns:
            (ndarray, ndarray) - 返回给定状态估计值的预测均值和协方差矩阵
        '''
        std = [self._std_weight_position * mean[3],
               self._std_weight_position * mean[3],
               1e-1,
               self._std_weight_position * mean[3]]
        innovation_cov = np.diag(np.square(std))
        mean = np.dot(self._update_mat, mean)
        covariance = np.linalg.multi_dot((self._update_mat, covariance, self._update_mat.T)) + innovation_cov

        return mean, covariance
  

    def update(self, mean, covariance, measurement):
        '''
        函数说明: 更新阶段函数
        Parameters:
            mean : ndarray - 预测状态的平均矢量(8维)
            covariance : ndarray - 该状态的协方差矩阵(8*8维)
            measurement : ndarray - (x, y, a, h) 中心位置 (x, y), 长宽比 a, 高度 h
        Returns:
            (ndarray, ndarray) - 返回测量校正后的状态分布
        '''
        projected_mean, projected_cov = self.project(mean, covariance)
        chol_factor, lower = scipy.linalg.cho_factor(projected_cov, lower=True, check_finite=False)
        kalman_gain = scipy.linalg.cho_solve((chol_factor, lower),
                                             np.dot(covariance, self._update_mat.T).T,
                                             check_finite=False).T
        innovation = measurement - projected_mean
        new_mean = mean + np.dot(innovation, kalman_gain.T)
        new_covariance = covariance - np.linalg.multi_dot((kalman_gain, projected_cov, kalman_gain.T))
        
        return new_mean, new_covariance


    def gating_distance(self, mean, covariance, measurements, only_position=False):
        '''
        函数说明: 计算状态分布检测框之间距离函数
        Parameters:
            mean : ndarray - 状态分布的平均向量(8维)
            covariance : ndarray - 状态分布的协方差矩阵(8*8维)
            measurements: ndarray - (x, y, a, h) 中心位置 (x, y), 长宽比 a, 高度 h
            only_position : Optional[bool] - 如果为 True, 距离计算将仅根据边界框中心位置进行
        Returns:
            ndarray - 返回长度为 N 的数组，其中第 i 个元素包含（均值、协方差）与测量值[i]之间的马哈拉诺比距离平方值。

        '''
        mean, covariance = self.project(mean, covariance)
        if only_position:
            mean, covariance = mean[:2], covariance[:2, :2]
            measurements = measurements[:, :2]
        cholesky_factor = np.linalg.cholesky(covariance)
        d = measurements - mean
        z = scipy.linalg.solve_triangular(cholesky_factor, d.T, lower=True,
                                          check_finite=False, overwrite_b=True
        squared_maha = np.sum(z * z, axis=0)
        
        return squared_maha
```
# 十、参考资料

1. https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
2. https://www.bilibili.com/video/BV1ez4y1X7eR/?spm_id_from=333.999.0.0&vd_source=c2f1782e251e8ca6ff9b6904b77a1d76
3. https://zhuanlan.zhihu.com/p/114516587