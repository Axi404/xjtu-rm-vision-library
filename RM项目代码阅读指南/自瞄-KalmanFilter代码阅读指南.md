前置：[卡尔曼滤波基础](拓展卡尔曼滤波(EKF))
## 综述
在xjtu-rm的自瞄中，涉及到EKF的代码主要有```KalmanFilter.hpp```,```KalmanFilter.cpp```,应用卡尔曼滤波的主要是```tracker.cpp/hpp```，用来预测追踪装甲板。
## Kalman类中的变量
首先我们看卡尔曼滤波的hpp头文件：
首先，代码定义了
```C++
using NonlinearFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
using JacobianFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
```
意思是把从向量到向量的函数为```NonlinearFunc```，后文可以看到主要是一个```f```一个```h```分别代表了由目前状态转移到下一时刻状态的非线性函数，和从目前状态转移到测量量的非线性函数.
另外一个向量到矩阵的函数```JacobianFunc```就是雅克布矩阵，存放着偏导，也就是EKF公式中会用到的$A$和$W$(具体可以见[这里](拓展卡尔曼滤波(EKF)##具体推导))。而在代码里对应着矩阵```F```和```H```.
一般计算方法是：
```C++
F=Jf(x_post);
H=Jh(x_post);
```
除此之外也定义了惯常的过程噪声分布协方差矩阵```Q```和观测噪声分布协方差矩阵```R```.

```C++
// Process nonlinear vector function
NonlinearFunc f;
// Observation nonlinear vector function
NonlinearFunc h;
// Jacobian of f()
JacobianFunc Jf;
Eigen::MatrixXd F;
// Jacobian of h()
JacobianFunc Jh;
Eigen::MatrixXd H;
// Process noise covariance matrix
Eigen::MatrixXd Q;
// Measurement noise covariance matrix
Eigen::MatrixXd R;
```
以上这些都是初始化时输入的.
接下来是一些关于状态，以及先验后验误差的变量：
```C++
// Priori error estimate covariance matrix
Eigen::MatrixXd P_pri;
// Posteriori error estimate covariance matrix
Eigen::MatrixXd P_post;
// Kalman gain
Eigen::MatrixXd K;
// System dimensions
int n;
// N-size identity
Eigen::MatrixXd I;
// Priori state
Eigen::VectorXd x_pri;
// Posteriori state
Eigen::VectorXd x_post;
```
下面是一个一一对应表格:

|  变量   | 数学公式  |  中文名  |
|  ----  | ----  | ----|
| ```P_pri```  | $P_k^-$ | 先验误差分布的协方差矩阵 |
| ```P_post```  | $P_k$ | 后验误差分布的协方差矩阵 |
| ```K``` | $k_K$ | 卡尔曼增益|
| ```n``` | / | 状态变量维度|
| ```I``` | $I$ |单位矩阵|
| ```x_pri``` | $\hat{x_k^-}$| 先验预测 |
| ```x_post```|$\hat{x_k}$ | 后验预测(最终结果)| 
如果忘记了具体含义可以去看[这篇](卡尔曼增益的推导).
贴上初始化函数：

```C++
ExtendedKalmanFilter::ExtendedKalmanFilter(
const NonlinearFunc &f, const NonlinearFunc &h, const JacobianFunc &Jf,
const JacobianFunc &Jh, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
const Eigen::MatrixXd &P0)
: f(f),
h(h),
Jf(Jf),
Jh(Jh),
Q(Q),
R(R),
P_post(P0),
n(Q.rows()),
I(Eigen::MatrixXd::Identity(n, n)),
x_pri(n),
x_post(n)
{
}
```
其中n就代表了状态向量的长度.
其中
```C++
x_pri(n),
x_post(n)
```
表示把```Eigen::VectorXd```长度定为```n```.

## Kalman类中的函数

### setState
```C++
void ExtendedKalmanFilter::setState(const Eigen::VectorXd &x0)
{
	x_post = x0;
}
```
用处是设置状态，可以应用于初始化等，传入一个向量，把后验预测设为该变量，以便作为当前程序预测的状态值参与下一次预测。
### predict和update
先看之前推导的计算流程：
![计算过程](拓展卡尔曼滤波(EKF)##计算流程)
在实际上的工程环境中，我们无法保证每一帧的测量都存在且有效，所以我们将```预测```和```更新```放在了两个函数中。
在实际应用中，我们采用如下策略：
```C++
//伪代码
ekf_prediction=predict();//先记录一个先验
if data 有效：
	ekf_post=update(data);//数据有效记录后验
else :
	处理数据异常;
```
所以我们先看```predict```函数:
```C++
Eigen::MatrixXd ExtendedKalmanFilter::predict()
{
x_pri = f(x_post);
F = Jf(x_post);
P_pri = F * P_post * F.transpose() + Q;
// handle the case when there will be no measurement before the next predict
x_post = x_pri;
P_post = P_pri;
return x_pri;
}
```
- 第一步 忽略误差直接套用函数算出来一个先验.
- 第二步 根据上一次的后验结果更新F的雅克布矩阵.
- 第三步 通过公式$P_k^-=FP_{k-1}F^T+WQW^T$计算出结果.
- 第四步 为了防止后续出现数据无效，提前有备无患，先把后验直接设置成先验。后续如果数据有效可以再更改.
- 返回先验预测值.

我们注意到，第三步中$WQW^T$直接写成了$Q$,这是因为$W=\frac{\delta f}{\delta w}$.而由于我们的模型比较简单，就可以直接把函数改写为$x_k=f(x_{k-1})+w_{k-1}$，我们这样的话，我们就会发现结果和$w_{k-1}$的关系恰好是斜率为1的直线，这样的话可以得出$W=I$.同理，$V=I$.
接下来看```update```函数:
```C++
Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd &z)
{
	H = Jh(x_pri);//存疑
	K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
	x_post = x_pri + K * (z - h(x_pri));
	P_post = (I - K * H) * P_pri;
	return x_post;
}
```
在已知测量值z的情况下，该函数做了以下操作：
- 根据x的先验（为什么不是上一次的后验呢？）算出H的雅克布矩阵.
- 根据公式$k_K=\frac{P_k^-H^T}{HP_k^-H^T+VRV^T}$算出卡尔曼增益.(为啥没有V的原因前面说了).
- 根据公式$\hat{x_k}=\hat{x_k^-}+k_K(z_k-h(\hat{x_k^-},0))$算出后验预测值.
- 根据公式$P_k=(I-k_KH)P_k^-$更新后验误差协方差矩阵.
## 附录-代码
头文件：
```C++
/*
infantry_9_28
*/
class ExtendedKalmanFilter
{
public:
ExtendedKalmanFilter() = default;
using NonlinearFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
using JacobianFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
explicit ExtendedKalmanFilter(
const NonlinearFunc &f, const NonlinearFunc &h, const JacobianFunc &Jf,
const JacobianFunc &Jh, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
const Eigen::MatrixXd &P0);
// Set the initial state
void setState(const Eigen::VectorXd &x0);
// Compute a predicted state
Eigen::MatrixXd predict();
// Update the estimated state based on measurement
Eigen::MatrixXd update(const Eigen::VectorXd &z);
private:
// Process nonlinear vector function
NonlinearFunc f;
// Observation nonlinear vector function
NonlinearFunc h;
// Jacobian of f()
JacobianFunc Jf;
Eigen::MatrixXd F;
// Jacobian of h()
JacobianFunc Jh;
Eigen::MatrixXd H;
// Process noise covariance matrix
Eigen::MatrixXd Q;
// Measurement noise covariance matrix
Eigen::MatrixXd R;
// Priori error estimate covariance matrix
Eigen::MatrixXd P_pri;
// Posteriori error estimate covariance matrix
Eigen::MatrixXd P_post;
// Kalman gain
Eigen::MatrixXd K;
// System dimensions
int n;
// N-size identity
Eigen::MatrixXd I;
// Priori state
Eigen::VectorXd x_pri;
// Posteriori state
Eigen::VectorXd x_post;

};
```
cpp实现：
```C++
ExtendedKalmanFilter::ExtendedKalmanFilter(
const NonlinearFunc &f, const NonlinearFunc &h, const JacobianFunc &Jf,
const JacobianFunc &Jh, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
const Eigen::MatrixXd &P0)
: f(f),
h(h),
Jf(Jf),
Jh(Jh),
Q(Q),
R(R),
P_post(P0),
n(Q.rows()),
I(Eigen::MatrixXd::Identity(n, n)),
x_pri(n),
x_post(n)
{
}
void ExtendedKalmanFilter::setState(const Eigen::VectorXd &x0)
{
x_post = x0;
}
Eigen::MatrixXd ExtendedKalmanFilter::predict()
{
x_pri = f(x_post);
F = Jf(x_post);
P_pri = F * P_post * F.transpose() + Q;
// handle the case when there will be no measurement before the next predict
x_post = x_pri;
P_post = P_pri;
return x_pri;
}
Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd &z)
{
H = Jh(x_pri);
K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
x_post = x_pri + K * (z - h(x_pri));
P_post = (I - K * H) * P_pri;
return x_post;
}
```
