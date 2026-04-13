<h1 align="center">Quadruped ROS2 Control</h1>

<p align="center">
  <img src="https://www.unitree.com/images/b5fffd3e4fc04e6f9fcafedb9516b341_3840x2146.jpg" alt="Unitree Go2 promotional image" width="88%">
</p>

<p align="center">
  <em>Unitree Go2 based simulation, controller debugging notes, and ROS 2 integration work.</em>
</p>

<p align="center">
  <a href="./readme.md">简体中文</a> | <strong>English</strong>
</p>

> A practical English walkthrough built on top of the original `Quadruped ROS2 Control` project. At this stage, the modifications mainly start from the `Unitree_guide` related part and are organized together with `Unitree Go2` simulation and debugging experience; further updates will gradually extend to modifications and optimization for other controllers as well.

---

## Branch Version
Before diving further into the current project, it is worth introducing an important branch version first: [`quadruped_ros2_control_unitree_guide` on the `debug` branch](https://github.com/yanyuze1/quadruped_ros2_control_unitree_guide/tree/debug?tab=readme-ov-file). This branch provides a more standalone extraction of the `unitree_guide` controller related part, making it easier to inspect, debug, and analyze that portion in a focused way. It also adds some debug-oriented content, which is especially helpful for troubleshooting, observing controller behavior, and supporting later optimization work.

## Preface
If you are also exploring quadruped control, simulation, and controller tuning, I hope this write-up helps you get oriented faster and avoid a few unnecessary detours. Special thanks to the [Quadruped ROS2 Control](https://github.com/legubiao/quadruped_ros2_control/tree/humble) project for providing such a solid foundation. This project is built and further refined on top of it, and the current work is based on the `Unitree_guide` branch. The sections below follow the real problems encountered during debugging and the ideas used to solve them, with the hope that the reading experience stays both useful and enjoyable all the way through.

## Results After Modification
### Simulation Results
![alt text](<images/2026-04-11 14-50-38.gif>)

This is the simulation result after the project modifications.

### Real Results

![alt text](images/bb601e9e3a84df7402d91245d37eb7bd.gif)

This is the real result after the project modifications，but it is not perfect.

## Project Modifications
Without changing the overall code structure, this project modifies the Unitree Guide controller part. When running simulation with the Unitree Guide controller, two issues appear:

### Issue 1: Limited In-Place Rotation in the Unitree Guide Controller
#### 1. Phenomenon

![alt text](<images/2026-04-11 15-15-24.gif>)

You can see that after rotating for some time, the robot dog performs an angle correction and returns toward the original heading.

#### 2. Cause of the Issue

In the original project, the turning input is essentially processed as "continuously tracking an accumulated `yaw_cmd_` angle" rather than "continuously rotating at a target yaw angular velocity". As a result, when the controller computes the attitude error:

```bash
rotMatToExp(Rd * G2B_RotMat)
```

it will always choose the shortest rotation path from the current pose to the target pose.

So when the robot keeps turning past a certain angle, the controller suddenly decides that "rotating in the opposite direction is closer", and starts outputting reverse z-axis angular acceleration and torque. It looks like the robot is being corrected back to the original heading.

#### 3. Related Files
* [StateTrotting.h](src/quadruped_ros2_control/controllers/unitree_guide_controller/include/unitree_guide_controller/FSM/StateTrotting.h)
* [StateTrotting.cpp](src/quadruped_ros2_control/controllers/unitree_guide_controller/src/FSM/StateTrotting.cpp)

#### 4. Analysis
There are mainly two layers to this problem.

1. Continuous turning was implemented as continuous integration of `yaw_cmd_`

The original idea is equivalent to:

```bash
yaw_cmd_ += d_yaw_cmd_ * dt_;
Rd = rotz(yaw_cmd_);
```

This means you are not telling the robot "keep turning at this angular velocity", but rather "keep chasing a heading that gets farther and farther away".

2. The attitude error uses the shortest rotation

In `mathTools.h`, the error is obtained by:

```bash
rotMatToExp(Rd * G2B_RotMat)
```

This expression naturally prefers the shortest rotation path from the current pose to the target pose. So when `yaw_cmd_` is continuously integrated, the error suddenly flips sign in a certain attitude range, and the controller outputs reverse z-axis angular acceleration and torque. That is why it looks like "once the robot rotates to a certain angle, it automatically corrects back".

#### 5. Solution Idea
For continuous in-place turning, the controller should use:

* While the stick is held: yaw rate control
Only control "how fast to turn", and stop chasing a continuously accumulated absolute yaw angle.
* After the stick is released: yaw hold
Lock the current yaw at the moment of release, then switch into heading hold.

This is why `yaw_rate_mode_` was introduced in the code. The core idea is just one sentence:

While the stick is held: only do yaw rate control. After the stick is released: do yaw hold.

#### 6. File Modifications for This Issue
1. Add 3 members in `StateTrotting.h`

```bash
bool yaw_rate_mode_{false};
bool yaw_rate_mode_last_{false};
double yaw_hold_deadband_{0.05};
```

Purpose:

* `yaw_rate_mode_`: whether the controller is currently in yaw-rate turning mode
* `yaw_rate_mode_last_`: used to determine whether it has just exited turning mode
* `yaw_hold_deadband_`: avoids noisy joystick input causing mode chattering

2. Initialize the current yaw in `enter()` inside `StateTrotting.cpp`

```bash
yaw_cmd_ = estimator_->getYaw();
yaw_cmd_ = std::atan2(std::sin(yaw_cmd_), std::cos(yaw_cmd_));
Rd = rotz(yaw_cmd_);
w_cmd_global_.setZero();

yaw_rate_mode_ = false;
yaw_rate_mode_last_ = false;
```

Purpose:

* The initial target heading is the current actual heading
* No unnecessary yaw error is introduced when first entering `TROTTING`

3. Convert `rx` into target yaw angular velocity in `getUserCmd()`

The current logic remains:

```bash
d_yaw_cmd_ = -invNormalize(ctrl_interfaces_.control_inputs_.rx, w_yaw_limit_(0), w_yaw_limit_(1));
d_yaw_cmd_ = 0.90 * d_yaw_cmd_past_ + (1 - 0.90) * d_yaw_cmd_;
d_yaw_cmd_past_ = d_yaw_cmd_;
```

Purpose:

* `rx` no longer directly controls the target angle
* It is first converted into the target yaw angular velocity `d_yaw_cmd_`

4. In `calcCmd()`, stop accumulating `yaw_cmd_` while the stick is held

Keep logic like this:

```bash
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

This is the most critical step.

Before the modification, the problematic logic was equivalent to:

```bash
yaw_cmd_ += d_yaw_cmd_ * dt_;
```

After the modification, it means:

* When the user is turning, `yaw_cmd_` no longer keeps growing through integration
* `yaw_cmd_` is directly locked to the current actual yaw
* This prevents the attitude controller from introducing extra yaw angle-position error
* The robot tracks angular velocity only through `w_cmd_global_(2)`
* After the stick is released, execute:

```bash
yaw_cmd_ = current_yaw;
```

This locks the heading at the instant the stick is released, and then enters heading hold.

5. Disable yaw position error in `calcTau()` during yaw-rate mode

```bash
Vec3 rot_error = rotMatToExp(Rd * G2B_RotMat);
if (yaw_rate_mode_) {
    rot_error(2) = 0.0;
}
```

Purpose:

* `rot_error(2)` is the yaw attitude error
* With continuous turning input, the controller should no longer chase yaw angle error
* Otherwise, even if `yaw_cmd_` is no longer integrated, the controller may still generate torque that pulls the robot back

6. Use the modified `rot_error` in `d_wbd`

```bash
Vec3 d_wbd = kp_w_ * rot_error +
             Kd_w_ * (w_cmd_global_ - estimator_->getGyroGlobal());
```

Purpose:

* Apply the modified attitude error directly in the calculation

### Issue 2: Coordinate Frame Problem in the Unitree Guide Controller
#### 1. Phenomenon

![alt text](<images/2026-04-11 21-11-31.gif>)

You can see that when the robot rotates to about `1.57 rad`, it loses balance and falls. If you observe the debug plots, you can clearly see a sudden torque jump.

#### 2. Cause of the Issue

* The coordinate frames of foot position and velocity observations in the estimator are inconsistent:

In `Estimator.cpp` and `Estimator.cpp`, the code directly inserts foot relative position and relative velocity expressed in the body frame into the observations. However, the observation matrix `C` in `Estimator.cpp` actually assumes world-frame relative quantities.

* This issue is not obvious when `yaw ~= 0`, but it becomes obvious at `1.57 rad`:

When `yaw ~= 1.57 rad`, close to `pi/2`, the body and world axes are nearly misaligned by 90 degrees. The error approaches its maximum there, so the problem is triggered reliably around that absolute heading.

* Swing leg velocity feedback uses an incorrect world-frame velocity:

In `Estimator.h`, `getFeetVel()` directly adds body-frame `J*qdot` to world-frame body velocity, missing both the `R` rotation and the `omega x r` term. As a result, the damping direction of the swing leg becomes more and more biased as the absolute yaw grows, and around `1.57 rad` the leg is most likely to be driven in the wrong direction.

```bash
Absolute yaw approaches about 1.57 rad
-> coordinate frame errors in foot position and velocity observations get amplified
-> swing leg velocity feedback points in the wrong direction
-> swing leg virtual force is applied in the wrong direction
-> the body starts to yaw/roll sideways
-> the QP balance controller tries to rescue it
-> yaw torque begins to jump back and forth
-> eventually the robot falls
```

#### 3. Related Files
* [Estimator.h](src/quadruped_ros2_control/controllers/unitree_guide_controller/include/unitree_guide_controller/control/Estimator.h)
* [Estimator.cpp](src/quadruped_ros2_control/controllers/unitree_guide_controller/src/control/Estimator.cpp)
* [StateTrotting.cpp](src/quadruped_ros2_control/controllers/unitree_guide_controller/src/FSM/StateTrotting.cpp)

#### 4. Analysis

1. The main cause is that the estimator treats body-frame foot observations as if they were world-frame observations.
The files are `Estimator.cpp` and `Estimator.h`.

2. In `update()` inside `Estimator.cpp`, `foot_poses_` and `foot_vels_` come from leg kinematics and are fundamentally body-frame quantities, but the model corresponding to the observation matrix `C` expects world-frame relative quantities.
In other words, the current observation model and the observation data are not in the same coordinate frame.

3. In `Estimator.h`, `getFeetVel()` directly does:

```bash
result.col(i) = Vec3(feet_vel[i].data) + getVelocity();
```

Here, body-frame `J*qdot` is directly added to world-frame body velocity. Two key terms are missing:

```bash
R * (...)
omega x r
```

4. When `yaw ~= 0`, the body/world direction difference is small, so the problem is not obvious. But when `yaw ~= 1.5 rad`, body x/y and world x/y are almost misaligned by 90 degrees, and this error becomes close to its maximum. That is why the robot consistently falls at a fixed absolute heading.

#### 5. Solution Idea
1. Fix the foot observation coordinate frame in the estimator, so foot relative position and relative velocity are converted into the world frame before entering the filter.
2. Fix `getFeetVel()`, so swing leg control uses the true world-frame foot velocity.
3. Fix the attitude control logic under `yaw_rate_mode_`, so yaw absolute angle error is truly disabled and only yaw angular velocity control remains.

#### 6. File Modifications for This Issue

1. Modify `getFeetVel()` in `Estimator.h`

Problem:

* `robot_model_->getFeet2BVelocities()` returns the relative foot-to-body velocity in the body frame.
* The current code directly treats it as a world-frame velocity.
* The correct world-frame foot velocity should satisfy:

```bash
v_foot_world = v_body_world + R * (omega_body x r_body + v_rel_body)
```

Replace `getFeetVel()` with:

```bash
Vec34 getFeetVel() {
    const std::vector<KDL::Vector> feet_vel_body_list = robot_model_->getFeet2BVelocities();
    const std::vector<KDL::Frame> feet_pos_body_list = robot_model_->getFeet2BPositions();
    Vec34 result;
    const Vec3 body_vel_world = getVelocity();

    for (int i(0); i < 4; ++i) {
        const Vec3 foot_pos_body = Vec3(feet_pos_body_list[i].p.data);
        const Vec3 foot_vel_body = Vec3(feet_vel_body_list[i].data);

        // [Modification 1]
        // World-frame foot velocity = body world velocity + R * (omega x r + v_rel_body)
        result.col(i) = body_vel_world +
                        rotation_ * (gyro_.cross(foot_pos_body) + foot_vel_body);
    }
    return result;
}
```

Why this works:

* `foot_vel_body` is only the leg's relative velocity in the body frame.
* When the robot body rotates, the foot additionally gains the `omega x r` term.
* Finally, it must be multiplied by `rotation_` to convert from the body frame to the world frame.
* After this fix, the velocity error direction used in swing-leg PD will no longer become severely distorted near absolute `yaw ~= 1.5 rad`.

2. Fix observation construction in `update()` inside `Estimator.cpp`

Problem:

* The code first obtains `foot_poses_` and `foot_vels_`, then directly puts them into `feet_pos_body_` and `feet_vel_body_`.
* These quantities are fundamentally body-frame quantities, but the `C` matrix in the Kalman filter corresponds to world-frame relative observations.
* So the IMU attitude and angular velocity need to be read first, and then the conversion must be applied.

It is recommended to replace the "read IMU + generate observations" part in `update()` with the following.

Original code:

```bash
foot_poses_ = robot_model_->getFeet2BPositions();
foot_vels_ = robot_model_->getFeet2BVelocities();
feet_h_.setZero();

for (...) {
    ...
    feet_pos_body_.segment(...) = Vec3(foot_poses_[i].p.data);
    feet_vel_body_.segment(...) = Vec3(foot_vels_[i].data);
}

Quat quat;
...
rotation_ = quatToRotMat(quat);

gyro_ << ...
acceleration_ << ...
```

Change it to:

```bash
foot_poses_ = robot_model_->getFeet2BPositions();
foot_vels_ = robot_model_->getFeet2BVelocities();
feet_h_.setZero();

// [Modification 2]
// Read IMU attitude and angular velocity first, because the foot observations
// below must be converted into the world frame consistently
Quat quat;
quat << ctrl_interfaces_.imu_state_interface_[0].get().get_value(),
        ctrl_interfaces_.imu_state_interface_[1].get().get_value(),
        ctrl_interfaces_.imu_state_interface_[2].get().get_value(),
        ctrl_interfaces_.imu_state_interface_[3].get().get_value();
rotation_ = quatToRotMat(quat);

gyro_ << ctrl_interfaces_.imu_state_interface_[4].get().get_value(),
        ctrl_interfaces_.imu_state_interface_[5].get().get_value(),
        ctrl_interfaces_.imu_state_interface_[6].get().get_value();

acceleration_ << ctrl_interfaces_.imu_state_interface_[7].get().get_value(),
        ctrl_interfaces_.imu_state_interface_[8].get().get_value(),
        ctrl_interfaces_.imu_state_interface_[9].get().get_value();

for (int i(0); i < 4; ++i) {
    if (wave_generator_->contact_[i] == 0) {
        Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) = large_variance_ * Eigen::MatrixXd::Identity(3, 3);
        R.block(12 + 3 * i, 12 + 3 * i, 3, 3) = large_variance_ * Eigen::MatrixXd::Identity(3, 3);
        R(24 + i, 24 + i) = large_variance_;
    } else {
        const double trust = windowFunc(wave_generator_->phase_[i], 0.2);
        Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) =
                (1 + (1 - trust) * large_variance_) *
                QInit_.block(6 + 3 * i, 6 + 3 * i, 3, 3);
        R.block(12 + 3 * i, 12 + 3 * i, 3, 3) =
                (1 + (1 - trust) * large_variance_) *
                RInit_.block(12 + 3 * i, 12 + 3 * i, 3, 3);
        R(24 + i, 24 + i) =
                (1 + (1 - trust) * large_variance_) * RInit_(24 + i, 24 + i);
    }

    const Vec3 foot_pos_body = Vec3(foot_poses_[i].p.data);
    const Vec3 foot_vel_body = Vec3(foot_vels_[i].data);

    // [Modification 2-1]
    // Convert the position observation into the world-frame relative foot-to-body position vector
    feet_pos_body_.segment(3 * i, 3) = rotation_ * foot_pos_body;

    // [Modification 2-2]
    // Convert the velocity observation into the world-frame relative foot-to-body velocity
    feet_vel_body_.segment(3 * i, 3) =
            rotation_ * (gyro_.cross(foot_pos_body) + foot_vel_body);
}
```

Why this works:

* Although the variable name `feet_pos_body_` contains `body`, once it is put into `y_`, it must satisfy the mathematical meaning required by observation matrix `C`, so it must be a world-frame relative position.
* The same applies to `feet_vel_body_`. It must be a world-frame relative velocity, otherwise the filter will trust incorrect observations and `x_hat_` will become systematically distorted near absolute `yaw ~= 1.5 rad`.
* IMU data must be read before the conversion because both `rotation_` and `gyro_` are used in the transform.

## Closing Notes
That is the complete set of fixes for this round of issues. This took me a long time to track down, but I finally found the root cause. I almost cried. I hope this helps future debugging when similar issues come up. There are still some problems on the real robot deployment side. Once I fix them, I will address them together. Later, I also plan to extract a standalone ROS2 version of `Unitree_guide` for everyone to use, and gradually add it back into the older simulation versions as well. Let's get started on reinforcement learning and the agent journey together.

![alt text](images/img_v3_0210l_6272228f-5c89-4187-a8d7-f1efbb24b1ag.jpg)

## Feishu Repository
Even though I do not understand everything, I will still tell you to keep going and give it your best.

![alt text](images/image.png)
