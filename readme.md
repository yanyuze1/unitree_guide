# Quadruped ROS2 Control 
 ## 前言 ## 
该本项目基于ROS2 Humble版本，项目基于彪哥的项目修改而来，感谢项目[Quadruped ROS2 Control](https://github.com/legubiao/quadruped_ros2_control/tree/humble)
## 项目修改 
在不改变代码结构的情况下对项目对Unitree Guide控制器部分进行了修改，在使用Unitree Guide控制器进行仿真时会出现两个问题：
### 问题1：Unitree Guide控制器原地旋转受限
#### 1.现象

#### 2.问题原因： 

原始项目里转向输入本质上被处理成了“不断追踪一个累加的 yaw_cmd_ 角度”，而不是“按一个目标 yaw 角速度持续旋转”。这样一来，控制器在计算姿态误差时：
``` bash
rotMatToExp(Rd * G2B_RotMat) 
```
会总是选择“当前姿态到目标姿态的最短旋转路径”。
所以当机器人持续转到一定角度后，控制器会突然认为“反方向更近”，于是开始输出反向 z 轴角加速度和力矩，看起来就像被矫正回原点
#### 3.问题文件：
* [StateTrotting.h](src/quadruped_ros2_control/controllers/unitree_guide_controller/include/unitree_guide_controller/FSM/StateTrotting.h)
* [StateTrotting.cpp](src/quadruped_ros2_control/controllers/unitree_guide_controller/src/FSM/StateTrotting.cpp)
#### 4.问题解析
问题主要有两层。

1. 把持续转向写成了持续积分 yaw_cmd_
原始思路等价于：
```bash
yaw_cmd_ += d_yaw_cmd_ * dt_;
Rd = rotz(yaw_cmd_);
```
这意味着你不是在告诉机器人“按这个角速度继续转”，而是在告诉机器人“不断去追一个越来越远的朝向”。

2. 姿态误差用的是最短旋转
在 mathTools.h 里，误差是通过：
```bash
rotMatToExp(Rd * G2B_RotMat)
```
得到的。这个表达天然会倾向于“当前姿态到目标姿态的最短旋转路径”。所以当你持续积分 yaw_cmd_ 后，误差到某个姿态区间会突然换符号，控制器就会给出反向的 z 轴角加速度和力矩，看起来就像“旋转到一定角度后自动纠回去”。
#### 5.解决思路
原地持续转向时，应该采用：
* 打杆时：yaw rate control
只控制“转多快”，不再追一个不断累加的绝对 yaw 角度
* 松杆时：yaw hold
把松杆那一刻的当前 yaw 锁住，再进入角度保持

这就是代码里引入 yaw_rate_mode_ 的意义。核心思想只有一句话：

打杆时：只做 yaw rate control，松杆时：再做 yaw hold
#### 6.问题文件修改
1.  在 StateTrotting.h 增加 3 个成员
``` bash
bool yaw_rate_mode_{false};
bool yaw_rate_mode_last_{false};
double yaw_hold_deadband_{0.05};
```
作用：

* yaw_rate_mode_：当前是否处于按角速度转向模式
* yaw_rate_mode_last_：用于判断是否刚从转向模式退出
* yaw_hold_deadband_：避免摇杆微小噪声导致模式抖动切换

2.  在 StateTrotting.cpp 的 enter() 中初始化当前 yaw
``` bash
yaw_cmd_ = estimator_->getYaw();
yaw_cmd_ = std::atan2(std::sin(yaw_cmd_), std::cos(yaw_cmd_));
Rd = rotz(yaw_cmd_);
w_cmd_global_.setZero();

yaw_rate_mode_ = false;
yaw_rate_mode_last_ = false;
```
作用：

* 初始目标朝向就是当前实际朝向
* 刚进入 TROTTING 时不会平白产生一个 yaw 误差

3. 在 getUserCmd() 中把 rx 转成目标 yaw 角速度

当前逻辑保留为：
```bash
d_yaw_cmd_ = -invNormalize(ctrl_interfaces_.control_inputs_.rx, w_yaw_limit_(0), w_yaw_limit_(1));
d_yaw_cmd_ = 0.90 * d_yaw_cmd_past_ + (1 - 0.90) * d_yaw_cmd_;
d_yaw_cmd_past_ = d_yaw_cmd_;
```
作用：

* rx 不再直接控制目标角度
* 而是先变成目标 yaw 角速度 d_yaw_cmd_

4. 在 calcCmd() 中，打杆时不要再累计 yaw_cmd_

保留这样的逻辑：
``` bash
const double current_yaw =
    std::atan2(std::sin(estimator_->getYaw()), std::cos(estimator_->getYaw()));

yaw_rate_mode_last_ = yaw_rate_mode_;
yaw_rate_mode_ = std::fabs(d_yaw_cmd_) > yaw_hold_deadband_;

if (yaw_rate_mode_) {
    yaw_cmd_ = current_yaw;
} else if (yaw_rate_mode_last_) {
    yaw_cmd_ = current_yaw;
}

Rd = rotz(yaw_cmd_);
w_cmd_global_.setZero();
w_cmd_global_(2) = d_yaw_cmd_;
```
这一步最关键。

修改前的问题写法等价于：
``` bash
yaw_cmd_ += d_yaw_cmd_ * dt_;
```
修改后的意思是：

* 当用户正在转向时，yaw_cmd_ 不再继续积分增长
* yaw_cmd_ 直接锁在当前实际 yaw
* 这样姿态控制器不会再制造额外的 yaw 角位置误差
* 机器人只通过 w_cmd_global_(2) 去跟踪角速度
* 松杆后再执行：
```bash
yaw_cmd_ = current_yaw;
```
这样就把松杆那一刻的朝向锁住，之后进入朝向保持

5. 在 calcTau() 中，yaw-rate 模式下关闭 yaw 位置误差
```bash
Vec3 rot_error = rotMatToExp(Rd * G2B_RotMat);
if (yaw_rate_mode_) {
    rot_error(2) = 0.0;
}
```
作用：

* rot_error(2) 是 yaw 姿态误差
* 有持续转向输入时，不应该再让控制器追 yaw 角度误差
* 否则即使前面不再积分 yaw_cmd_，后面依然可能产生“往回拉”的力矩

6. d_wbd 使用修改后的 rot_error

``` bash
Vec3 d_wbd = kp_w_ * rot_error +
             Kd_w_ * (w_cmd_global_ - estimator_->getGyroGlobal());
```
作用：
* 将修改后的姿态误差直接作用在计算中
### 问题2：Unitree Guide控制器坐标系
