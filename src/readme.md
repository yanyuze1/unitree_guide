# 下面是可提供进行debug的话题名
## BalanceCtrl (平衡控制层)

| 话题 (`Topic`) | 含义 | 作用 |
| :--- | :--- | :--- |
| `/unitree_guide/debug/balance/dd_pcd` | 机身期望线加速度 | 看平移控制目标是否过猛 |
| `/unitree_guide/debug/balance/d_wbd` | 机身期望角加速度 | 看姿态控制目标是否过猛 |
| `/unitree_guide/debug/balance/bd_force` | 理论期望总合力 | 看上层希望机身获得多大合力 |
| `/unitree_guide/debug/balance/bd_torque` | 理论期望总力矩 | 看上层希望机身获得多大力矩 |
| `/unitree_guide/debug/balance/force_fr` | FR 足端力 | 看右前腿是否异常失载或过载 |
| `/unitree_guide/debug/balance/force_fl` | FL 足端力 | 看左前腿是否异常失载或过载 |
| `/unitree_guide/debug/balance/force_rr` | RR 足端力 | 看右后腿是否异常失载或过载 |
| `/unitree_guide/debug/balance/force_rl` | RL 足端力 | 看左后腿是否异常失载或过载 |
| `/unitree_guide/debug/balance/solved_force` | 用 A*F 反算出的实际总合力 | 看 QP 真正实现了多少总力 |
| `/unitree_guide/debug/balance/solved_torque` | 用 A*F 反算出的实际总力矩 | 看 QP 真正实现了多少总力矩 |
| `/unitree_guide/debug/balance/force_error` | solved_force - bd_force | 看理论总合力和实际总合力差多少 |
| `/unitree_guide/debug/balance/torque_error` | solved_torque - bd_torque | 看理论总力矩和实际总力矩差多少 |
| `/unitree_guide/debug/balance/normal_force` | 四条腿法向力 Fz | 看哪条腿先失去支撑 |
| `/unitree_guide/debug/balance/min_constraint_margin` | 约束最小裕量 | 看摩擦锥或法向力约束是否被顶满 |

---

## Estimator (状态估计器)

| 话题 (`Topic`) | 含义 | 作用 |
| :--- | :--- | :--- |
| `/unitree_guide/debug/estimator/position` | 机身位置估计 | 看机身轨迹是否开始漂 |
| `/unitree_guide/debug/estimator/velocity` | 机身速度估计 | 看速度是否跟上输入 |
| `/unitree_guide/debug/estimator/gyro` | 机体系角速度 | 看 IMU 原始角速度 |
| `/unitree_guide/debug/estimator/gyro_global` | 世界系角速度 | 看滚转、俯仰、偏航哪一轴先爆 |
| `/unitree_guide/debug/estimator/acceleration` | 机体系线加速度 | 看 IMU 是否出现大冲击或抖动 |
| `/unitree_guide/debug/estimator/u_world` | 世界系输入加速度 | 看滤波器实际使用的加速度输入 |
| `/unitree_guide/debug/estimator/yaw` | 偏航角估计 | 看转向跟踪 |
| `/unitree_guide/debug/estimator/dyaw` | 偏航角速度估计 | 看转向响应 |
| `/unitree_guide/debug/estimator/contact` | 四脚触地状态 | 看支撑腿切换是否异常 |
| `/unitree_guide/debug/estimator/phase` | 四脚相位 | 看步态时序是否正常 |
| `/unitree_guide/debug/estimator/pos_meas_residual_norm` | 位置观测残差范数 | 看状态估计是否开始不可信 |
| `/unitree_guide/debug/estimator/vel_meas_residual_norm` | 速度观测残差范数 | 看状态估计是否开始不可信 |

---

## StateTrotting: 目标与误差 (小步态状态机)

| 话题 (`Topic`) | 含义 | 作用 |
| :--- | :--- | :--- |
| `/unitree_guide/debug/trotting/v_cmd_body` | 摇杆映射后的机体系速度指令 | 看输入是否正确进入控制器 |
| `/unitree_guide/debug/trotting/vel_target` | 世界系速度目标 | 看最终在追什么速度 |
| `/unitree_guide/debug/trotting/pos_body` | 当前机身位置 | 看当前状态 |
| `/unitree_guide/debug/trotting/vel_body` | 当前机身速度 | 看当前状态 |
| `/unitree_guide/debug/trotting/pcd` | 机身位置目标 | 看位置目标如何累积 |
| `/unitree_guide/debug/trotting/pos_error` | 位置误差 | 看机身位置是否跟不上 |
| `/unitree_guide/debug/trotting/vel_error` | 速度误差 | 看机身速度是否跟不上 |
| `/unitree_guide/debug/trotting/dd_pcd` | StateTrotting 算出的线加速度指令 | 看平移控制器输出 |
| `/unitree_guide/debug/trotting/rot_error` | 姿态误差 | 看机身姿态误差来源 |
| `/unitree_guide/debug/trotting/d_wbd` | StateTrotting 算出的角加速度指令 | 看姿态控制器输出 |
| `/unitree_guide/debug/trotting/gyro_global` | 当前世界系角速度和 d_wbd 对比 | 看是否跟上 |
| `/unitree_guide/debug/trotting/yaw_cmd` | 偏航目标 | 看转向目标 |
| `/unitree_guide/debug/trotting/yaw_est` | 偏航实际估计 | 看转向实际值 |
| `/unitree_guide/debug/trotting/yaw_error` | 偏航误差 | 看是否出现被拉回或跟不上 |
| `/unitree_guide/debug/trotting/dyaw_cmd` | 偏航角速度指令 | 看转向输入目标 |
| `/unitree_guide/debug/trotting/dyaw_est` | 偏航角速度实际估计 | 看转向响应 |
| `/unitree_guide/debug/trotting/wave_status` | 当前步态状态 | 看是否处在 STANCE_ALL/WAVE_ALL |

---

## StateTrotting: 足端与步态

| 话题 (`Topic`) | 含义 | 作用 |
| :--- | :--- | :--- |
| `/unitree_guide/debug/trotting/pos_feet_global_goal` | 四脚世界系目标位置 | 看落脚点目标是否合理 |
| `/unitree_guide/debug/trotting/vel_feet_global_goal` | 四脚世界系目标速度 | 看摆腿速度目标 |
| `/unitree_guide/debug/trotting/pos_feet_global` | 四脚世界系实际位置 | 看足端是否跟上目标 |
| `/unitree_guide/debug/trotting/vel_feet_global` | 四脚世界系实际速度 | 看足端是否跟上目标 |
| `/unitree_guide/debug/trotting/force_feet_global` | 四脚世界系足端力 | 看足端输出力 |
| `/unitree_guide/debug/trotting/force_feet_body` | 四脚机体系足端力 | 看送入逆动力学前的力 |
| `/unitree_guide/debug/trotting/phase` | 四脚相位 | 看摆动/支撑节奏 |
| `/unitree_guide/debug/trotting/contact` | 四脚触地状态 | 看支撑脚切换是否异常 |

---

## 关节执行层

| 话题 (`Topic`) | 含义 | 作用 |
| :--- | :--- | :--- |
| `/unitree_guide/debug/joint/q_goal` | 关节位置目标 | 看逆运动学输出 |
| `/unitree_guide/debug/joint/q_state` | 关节实际位置 | 看执行层实际状态 |
| `/unitree_guide/debug/joint/q_error` | 位置误差 | 看关节位置是否跟不上 |
| `/unitree_guide/debug/joint/qd_goal` | 关节速度目标 | 看速度目标 |
| `/unitree_guide/debug/joint/qd_state` | 关节实际速度 | 看执行层实际状态 |
| `/unitree_guide/debug/joint/qd_error` | 速度误差 | 看关节速度是否跟不上 |
| `/unitree_guide/debug/joint/tau_cmd` | 关节力矩命令 | 看最终下发给执行层的力矩 |
| `/unitree_guide/debug/joint/tau_state` | 关节实际力矩 | 看硬件或仿真实际输出的力矩 |
| `/unitree_guide/debug/joint/tau_error` | 力矩误差 | 看命令和实际执行差多少 |

---

## 辅助原生话题

| 话题 (`Topic`) | 含义 | 作用 |
| :--- | :--- | :--- |
| `/control_input` | 遥控输入 | 对齐所有曲线的起点 |
| `/joint_states` | 关节原生状态 | 和 `joint/*` debug 对照 |
| `/imu_sensor_broadcaster/imu` | IMU 原始消息 | 和 Estimator 估计对照 |