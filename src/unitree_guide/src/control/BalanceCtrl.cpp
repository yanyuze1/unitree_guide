//
// Created by tlab-uav on 24-9-16.
// QP平衡控制器的实现，在目标运动和约束条件下求解足端力(4条腿的X Y Z三方向)
// 目标是优雅的完成目标运动
//

#include "unitree_guide/control/BalanceCtrl.h"

#include <unitree_guide/common/mathTools.h>
#include <unitree_guide/robot/QuadrupedRobot.h>

#include "quadProgpp/QuadProg++.hh"

BalanceCtrl::BalanceCtrl(const std::shared_ptr<QuadrupedRobot> &robot) {
    mass_ = robot->mass_;

    alpha_ = 0.01;     // 足端力大小惩罚权重(尽量用较小的力完成动作，降低能耗)
    beta_ = 0.4;        // 足端力变化率惩罚权重（尽量保持输出顺滑）
    g_ << 0, 0, -9.81;
    friction_ratio_ = 0.5;  // 摩擦锥约束配置
    friction_mat_ << 1, 0, friction_ratio_, -1, 0, friction_ratio_, 0, 1, friction_ratio_, 0, -1,
            friction_ratio_, 0, 0, 1;   // 摩擦锥矩阵

    // 这里用的Go1的参数
    pcb_ = Vec3(-0.005, 0.0, 0.0);
    Ib_ = Vec3(0.0692, 0.1085, 0.1365).asDiagonal();

    Vec6 s;
    Vec12 w, u;
    w << 10, 10, 4, 10, 10, 4, 10, 10, 4, 10, 10, 4;
    u << 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3;
    s << 20, 20, 50, 450, 450, 450;     // s矩阵权重，对期望运动的影响(x，y,z,roll,pitch,yaw) 

    S_ = s.asDiagonal();
    W_ = w.asDiagonal();
    U_ = u.asDiagonal();

    F_prev_.setZero();
}

// ddPcd:目标机身加速度，dWbd：目标机身角加速度
// rot_matrix: 机身当前姿态，feet_pos_2_body：当前机身集合中心到各足端的向量在世界坐标。
Vec34 BalanceCtrl::calF(const Vec3 &ddPcd, const Vec3 &dWbd, const RotMat &rot_matrix,
                        const Vec34 &feet_pos_2_body, const VecInt4 &contact) {
    calMatrixA(feet_pos_2_body, rot_matrix);
    calVectorBd(ddPcd, dWbd, rot_matrix);
    calConstraints(contact);
    
    // 代价函数J，115页
    G_ = A_.transpose() * S_ * A_ + alpha_ * W_ + beta_ * U_; 
    g0T_ = -bd_.transpose() * S_ * A_ - beta_ * F_prev_.transpose() * U_;
    debug_data_.dd_pcd = ddPcd;
    debug_data_.d_wbd = dWbd;
    debug_data_.bd_force = bd_.head(3);
    debug_data_.bd_torque = bd_.tail(3);
    solveQP();
    const Vec34 force_feet = vec12ToVec34(F_);
    debug_data_.force = force_feet;
    const Vec6 solved_wrench = A_ * F_;
    debug_data_.solved_force = solved_wrench.head(3);
    debug_data_.solved_torque = solved_wrench.tail(3);
    debug_data_.force_error = debug_data_.solved_force - debug_data_.bd_force;
    debug_data_.torque_error = debug_data_.solved_torque - debug_data_.bd_torque;
    debug_data_.rot_matrix = rot_matrix;
    debug_data_.feet_pos_2_body = feet_pos_2_body;
    debug_data_.contact.setZero();
    for (int i = 0; i < 4; ++i) {
        debug_data_.contact(i) = static_cast<double>(contact[i]);
    }

    debug_data_.normal_force.setZero();
    for (int i = 0; i < 4; ++i) {
        debug_data_.normal_force(i) = F_(3 * i + 2);
    }

    if (CI_.rows() > 0) {
        const VecX margin = CI_ * F_ + ci0_;
        debug_data_.min_constraint_margin = margin.minCoeff();
    } else {
        debug_data_.min_constraint_margin = 0.0;
    }
    F_prev_ = F_;
    return vec12ToVec34(F_);
}

// 计算力/力矩的雅可比(映射)矩阵 A (维度 6 x 12)
void BalanceCtrl::calMatrixA(const Vec34 &feet_pos_2_body, const RotMat &rotM) {
    for (int i = 0; i < 4; ++i) {
        A_.block(0, 3 * i, 3, 3) = I3;
        A_.block(3, 3 * i, 3, 3) = skew(Vec3(feet_pos_2_body.col(i)) - rotM * pcb_);
    }
}

// 根据期望加速度计算期望躯干受力 Bd (维度 6 x 1)
void BalanceCtrl::calVectorBd(const Vec3 &ddPcd, const Vec3 &dWbd, const RotMat &rotM) {
    bd_.head(3) = mass_ * (ddPcd - g_);
    bd_.tail(3) = rotM * Ib_ * rotM.transpose() * dWbd;
}

// 构建物理约束矩阵
void BalanceCtrl::calConstraints(const VecInt4 &contact) {
    int contactLegNum = 0;
    for (int i(0); i < 4; ++i) {
        if (contact[i] == 1) {
            contactLegNum += 1;
        }
    }

    CI_.resize(5 * contactLegNum, 12);
    ci0_.resize(5 * contactLegNum);
    // 腾空腿不受力
    CE_.resize(3 * (4 - contactLegNum), 12);
    ce0_.resize(3 * (4 - contactLegNum));

    CI_.setZero();
    ci0_.setZero();
    CE_.setZero();
    ce0_.setZero();

    int ceID = 0;
    int ciID = 0;
    for (int i(0); i < 4; ++i) {
        if (contact[i] == 1) {
            CI_.block(5 * ciID, 3 * i, 5, 3) = friction_mat_;
            ++ciID;
        } else {
            CE_.block(3 * ceID, 3 * i, 3, 3) = I3;
            ++ceID;
        }
    }
}

// QP求解器
void BalanceCtrl::solveQP() {
    const long n = F_.size();
    const long m = ce0_.size();
    const long p = ci0_.size();

    // 声明 quadprogpp 所需的特定数据结构
    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, x;

    G.resize(n, n);
    CE.resize(n, m);
    CI.resize(n, p);
    g0.resize(n);
    ce0.resize(m);
    ci0.resize(p);
    x.resize(n);

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            G[i][j] = G_(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            CE[i][j] = CE_.transpose()(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < p; ++j) {
            CI[i][j] = CI_.transpose()(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        g0[i] = g0T_[i];
    }

    for (int i = 0; i < m; ++i) {
        ce0[i] = ce0_[i];
    }

    for (int i = 0; i < p; ++i) {
        ci0[i] = ci0_[i];
    }

    solve_quadprog(G, g0, CE, ce0, CI, ci0, x);

    for (int i = 0; i < n; ++i) {
        F_[i] = x[i];
    }
}
