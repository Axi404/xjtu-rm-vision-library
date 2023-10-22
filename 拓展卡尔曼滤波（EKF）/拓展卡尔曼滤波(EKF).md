前置：[误差协方差矩阵的推导](误差协方差矩阵的推导)
## 线性化，EKF！
- 拓展卡尔曼滤波是干啥的？
在上一节的学习中，我们知道了如何用卡尔曼滤波去算线性量的预测。但是如何处理非线性量？于是我们引入了拓展卡尔曼滤波（EKF）。
- 如何做到？
看似困难，实则大道至简：将非线性函数关于上一个时刻的后验预测值展开一阶线性项即可。
## 具体推导
$x_k=f(x_{k-1},\mu_{k-1},\omega_{k-1})$,$z_k=h(x_{k-1},v_{k-1})$
我们先以$x_k$的计算为例子：
首先我们先约定f的偏导，举个例子，假设x是二维变量，其中
$x_{k1}=f_1(x_{k-1},\mu_{k-1},\omega_{k-1})$
$x_{k2}=f_2(x_{k-1},\mu_{k-1},\omega_{k-1})$
那么$$\frac{\delta f}{\delta x}=\begin{bmatrix}\frac{\delta f_1}{\delta x_1}&\frac{\delta f_1}{\delta x_2}\\\frac{\delta f_2}{\delta x_1}&\frac{\delta f_2}{\delta x_2}\end{bmatrix}$$
其它情况同理。
那么我们就把$x_k$在$\hat{x_{k-1}}$处线性展开（因为我们不知道误差是多少，于是干脆把它看成0）：
$$x_k=f(\hat{x_{k-1}},\mu_{k-1},0)+A(x_{k-1}-\hat{x_{k-1}})+W\omega_{k-1}$$
其中$A=\frac{\delta f}{\delta x},W=\frac{\delta f}{\delta \omega}$,分别把$\hat{x_{k-1}},\mu_{k-1}$和$0$代入.
同理，我们把$z_k$也表达出来：
$$z_k=h(\hat{x_{k-1}},\mu_{k-1},0)+H(x_{k-1}-\hat{x_{k-1}})+Vv_{k-1}$$
$H,V$同理是偏导.
接下来求$W\omega_{k-1}$的协方差，公式略，结果为
$$P(W\omega_{k-1}) \sim N(0,W\omega_{k-1}\omega_{k-1}^TW^T)=N(0,WQW^T)$$
那么，我们可以把预测和校验过程修改为：
## 计算流程
### 计算矩阵
- 代入上一次的后验值，通过偏导算出来$A,H,V,W$.
### 预测
- 算先验
$$\hat{x_k^-}=f(\hat{x_{k-1}},\mu_{k-1},0)$$
- 算先验误差协方差
$$P_k^-=AP_{k-1}A^T+WQW^T$$
### 校正
- 算卡尔曼增益
$$k_K=\frac{P_k^-H^T}{HP_k^-H^T+VRV^T}$$
- 算后验估计
$$\hat{x_k}=\hat{x_k^-}+k_K(z_k-h(\hat{x_k^-},0))$$
- 更新误差协方差
$$P_k=(I-k_KH)P_k^-$$
完结撒花~
