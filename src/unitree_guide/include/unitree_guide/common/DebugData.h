#ifndef DEBUGDATA_H
#define DEBUGDATA_H

#include <unitree_guide/common/mathTypes.h>

struct BalanceCtrlDebugData {
    Vec3 dd_pcd;
    Vec3 d_wbd;
    Vec3 bd_force;
    Vec3 bd_torque;
    Vec34 force;

    Vec3 solved_force;
    Vec3 solved_torque;
    Vec3 force_error;
    Vec3 torque_error;
    Vec4 normal_force;
    Vec4 contact;
    Vec34 feet_pos_2_body;
    RotMat rot_matrix;
    double min_constraint_margin;

    BalanceCtrlDebugData() : min_constraint_margin(0.0) {
        dd_pcd.setZero();
        d_wbd.setZero();
        bd_force.setZero();
        bd_torque.setZero();
        force.setZero();
        solved_force.setZero();
        solved_torque.setZero();
        force_error.setZero();
        torque_error.setZero();
        normal_force.setZero();
        contact.setZero();
        rot_matrix.setZero();
        feet_pos_2_body.setZero();
    }
};

struct EstimatorDebugData {
    Vec3 position;
    Vec3 velocity;
    Vec3 gyro;
    Vec3 gyro_global;
    Vec3 acceleration;
    Vec3 u_world;
    Vec4 contact;
    Vec4 phase;
    double yaw;
    double dyaw;
    double pos_meas_residual_norm;
    double vel_meas_residual_norm;

    EstimatorDebugData()
        : yaw(0.0),
          dyaw(0.0),
          pos_meas_residual_norm(0.0),
          vel_meas_residual_norm(0.0) {
        position.setZero();
        velocity.setZero();
        gyro.setZero();
        gyro_global.setZero();
        acceleration.setZero();
        u_world.setZero();
        contact.setZero();
        phase.setZero();
    }
};

struct StateTrottingDebugData {
    Vec3 v_cmd_body;
    Vec3 vel_target;
    Vec3 pos_body;
    Vec3 vel_body;
    Vec3 pcd;
    Vec3 pos_error;
    Vec3 vel_error;
    Vec3 dd_pcd;
    Vec3 rot_error;
    Vec3 d_wbd;
    Vec3 gyro_global;

    Vec34 pos_feet_global_goal;
    Vec34 vel_feet_global_goal;
    Vec34 pos_feet_global;
    Vec34 vel_feet_global;
    Vec34 force_feet_global;
    Vec34 force_feet_body;

    Vec12 q_goal;
    Vec12 qd_goal;
    Vec12 tau_cmd;
    Vec12 q_state;
    Vec12 qd_state;
    Vec12 tau_state;
    Vec12 q_error;
    Vec12 qd_error;
    Vec12 tau_error;

    Vec4 phase;
    Vec4 contact;

    double yaw_cmd;
    double yaw_est;
    double yaw_error;
    double d_yaw_cmd;
    double d_yaw_est;
    int wave_status;

    StateTrottingDebugData()
        : yaw_cmd(0.0),
          yaw_est(0.0),
          yaw_error(0.0),
          d_yaw_cmd(0.0),
          d_yaw_est(0.0),
          wave_status(0) {
        v_cmd_body.setZero();
        vel_target.setZero();
        pos_body.setZero();
        vel_body.setZero();
        pcd.setZero();
        pos_error.setZero();
        vel_error.setZero();
        dd_pcd.setZero();
        rot_error.setZero();
        d_wbd.setZero();
        gyro_global.setZero();

        pos_feet_global_goal.setZero();
        vel_feet_global_goal.setZero();
        pos_feet_global.setZero();
        vel_feet_global.setZero();
        force_feet_global.setZero();
        force_feet_body.setZero();

        q_goal.setZero();
        qd_goal.setZero();
        tau_cmd.setZero();
        q_state.setZero();
        qd_state.setZero();
        tau_state.setZero();
        q_error.setZero();
        qd_error.setZero();
        tau_error.setZero();

        phase.setZero();
        contact.setZero();
    }
};

#endif // DEBUGDATA_H
