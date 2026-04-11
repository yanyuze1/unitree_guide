//
// Created by biao on 24-9-18.
// 计算当前机器人状态下目标落脚点坐标
//

#include "unitree_guide/gait/FeetEndCalc.h"

#include <unitree_guide/control/CtrlComponent.h>
#include <unitree_guide/control/Estimator.h>

FeetEndCalc::FeetEndCalc(CtrlComponent &ctrl_component)
    : ctrl_component_(ctrl_component),
      robot_model_(ctrl_component.robot_model_),
      estimator_(ctrl_component.estimator_) {
    // 落足点反馈控制增益
    k_x_ = 0.005;
    k_y_ = 0.005;
    k_yaw_ = 0.010;
}

void FeetEndCalc::init() {
    // 从波形发生器中获取stance(支撑相时间)和swing(摆动相时间)
    t_stance_ = ctrl_component_.wave_generator_->get_t_stance();
    t_swing_ = ctrl_component_.wave_generator_->get_t_swing();

    // 获取当前4条腿相对机器人body坐标系位置
    // Vec34 feet_pos_body = estimator_->getFeetPos2Body();
    Vec34 feet_pos_body = robot_model_->feet_pos_normal_stand_;
    for (int i(0); i < 4; ++i) {
        feet_radius_(i) =
                sqrt(pow(feet_pos_body(0, i), 2) + pow(feet_pos_body(1, i), 2));
        feet_init_angle_(i) = atan2(feet_pos_body(1, i), feet_pos_body(0, i));
    }
}

Vec3 FeetEndCalc::calcFootPos(const int index, Vec2 vxy_goal_global, const double d_yaw_global, const double phase) {
    Vec3 body_vel_global = estimator_->getVelocity();
    Vec3 next_step;

    next_step(0) = body_vel_global(0) * (1 - phase) * t_swing_ +
                   body_vel_global(0) * t_stance_ / 2 +
                   k_x_ * (body_vel_global(0) - vxy_goal_global(0));
    next_step(1) = body_vel_global(1) * (1 - phase) * t_swing_ +
                   body_vel_global(1) * t_stance_ / 2 +
                   k_y_ * (body_vel_global(1) - vxy_goal_global(1));
    next_step(2) = 0;

    const double yaw = estimator_->getYaw();
    const double d_yaw = estimator_->getDYaw();
    const double next_yaw = d_yaw * (1 - phase) * t_swing_ + d_yaw * t_stance_ / 2 +
                            k_yaw_ * (d_yaw_global - d_yaw);

    next_step(0) +=
            feet_radius_(index) * cos(yaw + feet_init_angle_(index) + next_yaw);
    next_step(1) +=
            feet_radius_(index) * sin(yaw + feet_init_angle_(index) + next_yaw);

    Vec3 foot_pos = estimator_->getPosition() + next_step;
    foot_pos(2) = 0.0;

    return foot_pos;
}
