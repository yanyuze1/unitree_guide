/**
 * control_input + joint/imu state -> UnitreeGuideController::update() -> FSM状态(run) -> BalanceCtrl / Gait / IK -> joint torque/position/velocity/kp/kd
 * 从硬件/仿真里读取关节和 IMU 状态，接收遥控输入，再通过状态机、状态估计、步态生成和平衡控制，最终给 12 个关节输出 effort / position / velocity / kp / kd 命令。
 * 核心算法与硬件/仿真链接的接口在 UnitreeGuideController 类中实现，主要包括以下几个部分：
 * 1. 接口配置：定义了控制器需要的命令接口和状态接口，包括关节状态、IMU 状态和控制输入等。
 * 2. 状态机实现：通过 FSMState 类和 FSMStateName 枚举定义了不同的状态，如被动状态、固定站立、行走等，并在 update() 方法中根据当前状态和输入进行状态转换。
 * 3. 控制算法：在不同的状态中调用 BalanceCtrl 类进行平衡控制，调用 WaveGenerator 类进行步态生成，并根据状态获取步态参数，并调用 BalanceCtrl 类进行控制
**/

#include "unitree_guide/UnitreeGuideController.h"

#include <unitree_guide/gait/WaveGenerator.h>
#include <unitree_guide/robot/QuadrupedRobot.h>

#include <algorithm>
#include <array>
#include <string>

namespace
{
geometry_msgs::msg::Vector3Stamped makeVec3Msg(
    const rclcpp::Time &stamp, const std::string &frame_id, const Vec3 &v)
{
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.vector.x = v(0);
    msg.vector.y = v(1);
    msg.vector.z = v(2);
    return msg;
}
std_msgs::msg::Float64 makeScalarMsg(double value)
{
    std_msgs::msg::Float64 msg;
    msg.data = value;
    return msg;
}

std_msgs::msg::Float64MultiArray makeArrayMsg(const Vec4 &v)
{
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(4);
    for (int i = 0; i < 4; ++i) {
        msg.data[static_cast<size_t>(i)] = v(i);
    }
    return msg;
}

std_msgs::msg::Float64MultiArray makeArrayMsg(const Vec12 &v)
{
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(12);
    for (int i = 0; i < 12; ++i) {
        msg.data[static_cast<size_t>(i)] = v(i);
    }
    return msg;
}

std_msgs::msg::Float64MultiArray makeArrayMsg(const Vec34 &m)
{
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(12);
    for (int leg = 0; leg < 4; ++leg) {
        for (int axis = 0; axis < 3; ++axis) {
            msg.data[static_cast<size_t>(leg * 3 + axis)] = m(axis, leg);
        }
    }
    return msg;
}

std_msgs::msg::Float64MultiArray makeArrayMsg(const RotMat &m)
{
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(9);
    int k = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            msg.data[static_cast<size_t>(k++)] = m(i, j);
        }
    }
    return msg;
}
}

namespace unitree_guide_controller
{
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration UnitreeGuideController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : command_interface_types_)
            {
                if (!command_prefix_.empty())
                {
                    conf.names.push_back(command_prefix_ + "/" + joint_name + "/" += interface_type);
                }
                else
                {
                    conf.names.push_back(joint_name + "/" += interface_type);
                }
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration UnitreeGuideController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }

        for (const auto& interface_type : imu_interface_types_)
        {
            conf.names.push_back(imu_name_ + "/" += interface_type);
        }

        return conf;
    }

    controller_interface::return_type UnitreeGuideController::
    update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // auto now = std::chrono::steady_clock::now();
        // std::chrono::duration<double> time_diff = now - last_update_time_;
        // last_update_time_ = now;
        //
        // // Calculate the frequency
        // update_frequency_ = 1.0 / time_diff.count();
        // RCLCPP_INFO(get_node()->get_logger(), "Update frequency: %f Hz", update_frequency_);

        if (ctrl_component_.robot_model_ == nullptr)
        {
            return controller_interface::return_type::OK;
        }

        ctrl_component_.robot_model_->update();
        ctrl_component_.wave_generator_->update();
        ctrl_component_.estimator_->update();

        if (mode_ == FSMMode::NORMAL)
        {
            current_state_->run(time, period);
            next_state_name_ = current_state_->checkChange();
            if (next_state_name_ != current_state_->state_name)
            {
                mode_ = FSMMode::CHANGE;
                next_state_ = getNextState(next_state_name_);
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
            }
        }
        else if (mode_ == FSMMode::CHANGE)
        {
            current_state_->exit();
            current_state_ = next_state_;

            current_state_->enter();
            mode_ = FSMMode::NORMAL;
        }
        if (enable_debug_topics_ &&
            ctrl_component_.balance_ctrl_ != nullptr &&
            ctrl_component_.estimator_ != nullptr)
        {
            ++debug_topic_counter_;
            if (debug_topic_counter_ >= std::max(1, debug_topic_decimation_))
            {
                debug_topic_counter_ = 0;
                publishDebugTopics(time);
            }
        }
        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn UnitreeGuideController::on_init()
    {
        try
        {
            joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
            command_interface_types_ =
                auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            state_interface_types_ =
                auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

            // imu sensor
            imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
            base_name_ = auto_declare<std::string>("base_name", base_name_);
            imu_interface_types_ = auto_declare<std::vector<std::string>>("imu_interfaces", state_interface_types_);
            command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
            feet_names_ =
                auto_declare<std::vector<std::string>>("feet_names", feet_names_);

            // pose parameters
            down_pos_ = auto_declare<std::vector<double>>("down_pos", down_pos_);
            stand_pos_ = auto_declare<std::vector<double>>("stand_pos", stand_pos_);
            stand_kp_ = auto_declare<double>("stand_kp", stand_kp_);
            stand_kd_ = auto_declare<double>("stand_kd", stand_kd_);

            enable_debug_topics_ = auto_declare<bool>("debug.enable_topics", true);
            debug_topic_decimation_ = auto_declare<int>("debug.decimation", 4);

            get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
            RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz", ctrl_interfaces_.frequency_);

            ctrl_component_.estimator_ = std::make_shared<Estimator>(ctrl_interfaces_, ctrl_component_);
        }
        catch (const std::exception& e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UnitreeGuideController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        if (enable_debug_topics_)
        {
            const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

            auto make_vec3_pub = [&](auto &pub, const std::string &name) {
                pub = get_node()->create_publisher<geometry_msgs::msg::Vector3Stamped>(name, qos);
            };
            auto make_scalar_pub = [&](auto &pub, const std::string &name) {
                pub = get_node()->create_publisher<std_msgs::msg::Float64>(name, qos);
            };
            auto make_array_pub = [&](auto &pub, const std::string &name) {
                pub = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(name, qos);
            };

            make_vec3_pub(balance_dd_pcd_pub_, "/unitree_guide/debug/balance/dd_pcd");
            make_vec3_pub(balance_d_wbd_pub_, "/unitree_guide/debug/balance/d_wbd");
            make_vec3_pub(balance_bd_force_pub_, "/unitree_guide/debug/balance/bd_force");
            make_vec3_pub(balance_bd_torque_pub_, "/unitree_guide/debug/balance/bd_torque");
            make_vec3_pub(balance_force_pub_[0], "/unitree_guide/debug/balance/force_fr");
            make_vec3_pub(balance_force_pub_[1], "/unitree_guide/debug/balance/force_fl");
            make_vec3_pub(balance_force_pub_[2], "/unitree_guide/debug/balance/force_rr");
            make_vec3_pub(balance_force_pub_[3], "/unitree_guide/debug/balance/force_rl");

            make_vec3_pub(balance_solved_force_pub_, "/unitree_guide/debug/balance/solved_force");
            make_vec3_pub(balance_solved_torque_pub_, "/unitree_guide/debug/balance/solved_torque");
            make_vec3_pub(balance_force_error_pub_, "/unitree_guide/debug/balance/force_error");
            make_vec3_pub(balance_torque_error_pub_, "/unitree_guide/debug/balance/torque_error");
            make_array_pub(balance_normal_force_pub_, "/unitree_guide/debug/balance/normal_force");
            make_scalar_pub(
                balance_min_constraint_margin_pub_,
                "/unitree_guide/debug/balance/min_constraint_margin");
            make_array_pub(balance_contact_pub_, "/unitree_guide/debug/balance/contact");
            make_array_pub(balance_rot_matrix_pub_, "/unitree_guide/debug/balance/rot_matrix");
            make_array_pub(balance_feet_pos_2_body_pub_, "/unitree_guide/debug/balance/feet_pos_2_body");


            make_vec3_pub(estimator_position_pub_, "/unitree_guide/debug/estimator/position");
            make_vec3_pub(estimator_velocity_pub_, "/unitree_guide/debug/estimator/velocity");
            make_vec3_pub(estimator_gyro_pub_, "/unitree_guide/debug/estimator/gyro");
            make_vec3_pub(estimator_gyro_global_pub_, "/unitree_guide/debug/estimator/gyro_global");
            make_vec3_pub(estimator_acceleration_pub_, "/unitree_guide/debug/estimator/acceleration");
            make_vec3_pub(estimator_u_world_pub_, "/unitree_guide/debug/estimator/u_world");
            make_scalar_pub(estimator_yaw_pub_, "/unitree_guide/debug/estimator/yaw");
            make_scalar_pub(estimator_dyaw_pub_, "/unitree_guide/debug/estimator/dyaw");
            make_array_pub(estimator_contact_pub_, "/unitree_guide/debug/estimator/contact");
            make_array_pub(estimator_phase_pub_, "/unitree_guide/debug/estimator/phase");
            make_scalar_pub(
                estimator_pos_meas_residual_norm_pub_,
                "/unitree_guide/debug/estimator/pos_meas_residual_norm");
            make_scalar_pub(
                estimator_vel_meas_residual_norm_pub_,
                "/unitree_guide/debug/estimator/vel_meas_residual_norm");

            make_vec3_pub(trotting_v_cmd_body_pub_, "/unitree_guide/debug/trotting/v_cmd_body");
            make_vec3_pub(trotting_vel_target_pub_, "/unitree_guide/debug/trotting/vel_target");
            make_vec3_pub(trotting_pos_body_pub_, "/unitree_guide/debug/trotting/pos_body");
            make_vec3_pub(trotting_vel_body_pub_, "/unitree_guide/debug/trotting/vel_body");
            make_vec3_pub(trotting_pcd_pub_, "/unitree_guide/debug/trotting/pcd");
            make_vec3_pub(trotting_pos_error_pub_, "/unitree_guide/debug/trotting/pos_error");
            make_vec3_pub(trotting_vel_error_pub_, "/unitree_guide/debug/trotting/vel_error");
            make_vec3_pub(trotting_dd_pcd_pub_, "/unitree_guide/debug/trotting/dd_pcd");
            make_vec3_pub(trotting_rot_error_pub_, "/unitree_guide/debug/trotting/rot_error");
            make_vec3_pub(trotting_d_wbd_pub_, "/unitree_guide/debug/trotting/d_wbd");
            make_vec3_pub(trotting_gyro_global_pub_, "/unitree_guide/debug/trotting/gyro_global");

            make_array_pub(
                trotting_pos_feet_global_goal_pub_,
                "/unitree_guide/debug/trotting/pos_feet_global_goal");
            make_array_pub(
                trotting_vel_feet_global_goal_pub_,
                "/unitree_guide/debug/trotting/vel_feet_global_goal");
            make_array_pub(
                trotting_pos_feet_global_pub_,
                "/unitree_guide/debug/trotting/pos_feet_global");
            make_array_pub(
                trotting_vel_feet_global_pub_,
                "/unitree_guide/debug/trotting/vel_feet_global");
            make_array_pub(
                trotting_force_feet_global_pub_,
                "/unitree_guide/debug/trotting/force_feet_global");
            make_array_pub(
                trotting_force_feet_body_pub_,
                "/unitree_guide/debug/trotting/force_feet_body");

            make_array_pub(joint_q_goal_pub_, "/unitree_guide/debug/joint/q_goal");
            make_array_pub(joint_qd_goal_pub_, "/unitree_guide/debug/joint/qd_goal");
            make_array_pub(joint_tau_cmd_pub_, "/unitree_guide/debug/joint/tau_cmd");
            make_array_pub(joint_q_state_pub_, "/unitree_guide/debug/joint/q_state");
            make_array_pub(joint_qd_state_pub_, "/unitree_guide/debug/joint/qd_state");
            make_array_pub(joint_tau_state_pub_, "/unitree_guide/debug/joint/tau_state");
            make_array_pub(joint_q_error_pub_, "/unitree_guide/debug/joint/q_error");
            make_array_pub(joint_qd_error_pub_, "/unitree_guide/debug/joint/qd_error");
            make_array_pub(joint_tau_error_pub_, "/unitree_guide/debug/joint/tau_error");

            make_array_pub(trotting_phase_pub_, "/unitree_guide/debug/trotting/phase");
            make_array_pub(trotting_contact_pub_, "/unitree_guide/debug/trotting/contact");
            make_scalar_pub(trotting_yaw_cmd_pub_, "/unitree_guide/debug/trotting/yaw_cmd");
            make_scalar_pub(trotting_yaw_est_pub_, "/unitree_guide/debug/trotting/yaw_est");
            make_scalar_pub(trotting_yaw_error_pub_, "/unitree_guide/debug/trotting/yaw_error");
            make_scalar_pub(trotting_dyaw_cmd_pub_, "/unitree_guide/debug/trotting/dyaw_cmd");
            make_scalar_pub(trotting_dyaw_est_pub_, "/unitree_guide/debug/trotting/dyaw_est");
            make_scalar_pub(trotting_wave_status_pub_, "/unitree_guide/debug/trotting/wave_status");
        }

        control_input_subscription_ = get_node()->create_subscription<control_input_msgs::msg::Inputs>(
            "/control_input", 10, [this](const control_input_msgs::msg::Inputs::SharedPtr msg)
            {
                // Handle message
                ctrl_interfaces_.control_inputs_.command = msg->command;
                ctrl_interfaces_.control_inputs_.lx = msg->lx;
                ctrl_interfaces_.control_inputs_.ly = msg->ly;
                ctrl_interfaces_.control_inputs_.rx = msg->rx;
                ctrl_interfaces_.control_inputs_.ry = msg->ry;
            });

        robot_description_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                ctrl_component_.robot_model_ = std::make_shared<QuadrupedRobot>(
                    ctrl_interfaces_, msg->data, feet_names_, base_name_);
                ctrl_component_.balance_ctrl_ = std::make_shared<BalanceCtrl>(ctrl_component_.robot_model_);
            });

        ctrl_component_.wave_generator_ = std::make_shared<WaveGenerator>(0.55, 0.6, Vec4(0, 0.5, 0.5, 0));

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    UnitreeGuideController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // clear out vectors in case of restart
        ctrl_interfaces_.clear();

        // assign command interfaces
        for (auto& interface : command_interfaces_)
        {
            std::string interface_name = interface.get_interface_name();
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos)
            {
                command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
            }
            else
            {
                command_interface_map_[interface_name]->push_back(interface);
            }
        }

        // assign state interfaces
        for (auto& interface : state_interfaces_)
        {
            if (interface.get_prefix_name() == imu_name_)
            {
                ctrl_interfaces_.imu_state_interface_.emplace_back(interface);
            }
            else
            {
                state_interface_map_[interface.get_interface_name()]->push_back(interface);
            }
        }

        // Create FSM List
        state_list_.passive = std::make_shared<StatePassive>(ctrl_interfaces_);
        state_list_.fixedDown = std::make_shared<StateFixedDown>(ctrl_interfaces_, down_pos_, stand_kp_, stand_kd_);
        state_list_.fixedStand = std::make_shared<StateFixedStand>(ctrl_interfaces_, stand_pos_, stand_kp_, stand_kd_);
        state_list_.swingTest = std::make_shared<StateSwingTest>(ctrl_interfaces_, ctrl_component_);
        state_list_.freeStand = std::make_shared<StateFreeStand>(ctrl_interfaces_, ctrl_component_);
        state_list_.balanceTest = std::make_shared<StateBalanceTest>(ctrl_interfaces_, ctrl_component_);
        state_list_.trotting = std::make_shared<StateTrotting>(ctrl_interfaces_, ctrl_component_);

        // Initialize FSM
        current_state_ = state_list_.passive;
        current_state_->enter();
        next_state_ = current_state_;
        next_state_name_ = current_state_->state_name;
        mode_ = FSMMode::NORMAL;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UnitreeGuideController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    UnitreeGuideController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    UnitreeGuideController::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    UnitreeGuideController::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    void UnitreeGuideController::publishDebugTopics(const rclcpp::Time &time)
    {
        if (ctrl_component_.balance_ctrl_ == nullptr || ctrl_component_.estimator_ == nullptr) {
            return;
        }

        const auto &bal = ctrl_component_.balance_ctrl_->getDebugData();
        const auto &est = ctrl_component_.estimator_->getDebugData();

        auto pub_vec3 = [&](const auto &pub, const std::string &frame_id, const Vec3 &v) {
            if (pub) {
                pub->publish(makeVec3Msg(time, frame_id, v));
            }
        };
        auto pub_scalar = [&](const auto &pub, double value) {
            if (pub) {
                pub->publish(makeScalarMsg(value));
            }
        };
        auto pub_v4 = [&](const auto &pub, const Vec4 &v) {
            if (pub) {
                pub->publish(makeArrayMsg(v));
            }
        };
        auto pub_v12 = [&](const auto &pub, const Vec12 &v) {
            if (pub) {
                pub->publish(makeArrayMsg(v));
            }
        };
        auto pub_v34 = [&](const auto &pub, const Vec34 &v) {
            if (pub) {
                pub->publish(makeArrayMsg(v));
            }
        };

        pub_vec3(balance_dd_pcd_pub_, "world", bal.dd_pcd);
        pub_vec3(balance_d_wbd_pub_, "world", bal.d_wbd);
        pub_vec3(balance_bd_force_pub_, "world", bal.bd_force);
        pub_vec3(balance_bd_torque_pub_, "world", bal.bd_torque);
        pub_vec3(balance_force_pub_[0], "world", bal.force.col(0));
        pub_vec3(balance_force_pub_[1], "world", bal.force.col(1));
        pub_vec3(balance_force_pub_[2], "world", bal.force.col(2));
        pub_vec3(balance_force_pub_[3], "world", bal.force.col(3));
        pub_vec3(balance_solved_force_pub_, "world", bal.solved_force);
        pub_vec3(balance_solved_torque_pub_, "world", bal.solved_torque);
        pub_vec3(balance_force_error_pub_, "world", bal.force_error);
        pub_vec3(balance_torque_error_pub_, "world", bal.torque_error);
        pub_v4(balance_normal_force_pub_, bal.normal_force);
        pub_scalar(balance_min_constraint_margin_pub_, bal.min_constraint_margin);
        pub_v4(balance_contact_pub_, bal.contact);
        if (balance_rot_matrix_pub_) {
            balance_rot_matrix_pub_->publish(makeArrayMsg(bal.rot_matrix));
        }
        pub_v34(balance_feet_pos_2_body_pub_, bal.feet_pos_2_body);


        pub_vec3(estimator_position_pub_, "world", est.position);
        pub_vec3(estimator_velocity_pub_, "world", est.velocity);
        pub_vec3(estimator_gyro_pub_, base_name_, est.gyro);
        pub_vec3(estimator_gyro_global_pub_, "world", est.gyro_global);
        pub_vec3(estimator_acceleration_pub_, base_name_, est.acceleration);
        pub_vec3(estimator_u_world_pub_, "world", est.u_world);
        pub_scalar(estimator_yaw_pub_, est.yaw);
        pub_scalar(estimator_dyaw_pub_, est.dyaw);
        pub_v4(estimator_contact_pub_, est.contact);
        pub_v4(estimator_phase_pub_, est.phase);
        pub_scalar(estimator_pos_meas_residual_norm_pub_, est.pos_meas_residual_norm);
        pub_scalar(estimator_vel_meas_residual_norm_pub_, est.vel_meas_residual_norm);

        if (current_state_ != nullptr &&
            current_state_->state_name == FSMStateName::TROTTING &&
            state_list_.trotting != nullptr) {

            const auto &tr = state_list_.trotting->getDebugData();

            pub_vec3(trotting_v_cmd_body_pub_, base_name_, tr.v_cmd_body);
            pub_vec3(trotting_vel_target_pub_, "world", tr.vel_target);
            pub_vec3(trotting_pos_body_pub_, "world", tr.pos_body);
            pub_vec3(trotting_vel_body_pub_, "world", tr.vel_body);
            pub_vec3(trotting_pcd_pub_, "world", tr.pcd);
            pub_vec3(trotting_pos_error_pub_, "world", tr.pos_error);
            pub_vec3(trotting_vel_error_pub_, "world", tr.vel_error);
            pub_vec3(trotting_dd_pcd_pub_, "world", tr.dd_pcd);
            pub_vec3(trotting_rot_error_pub_, base_name_, tr.rot_error);
            pub_vec3(trotting_d_wbd_pub_, "world", tr.d_wbd);
            pub_vec3(trotting_gyro_global_pub_, "world", tr.gyro_global);

            pub_v34(trotting_pos_feet_global_goal_pub_, tr.pos_feet_global_goal);
            pub_v34(trotting_vel_feet_global_goal_pub_, tr.vel_feet_global_goal);
            pub_v34(trotting_pos_feet_global_pub_, tr.pos_feet_global);
            pub_v34(trotting_vel_feet_global_pub_, tr.vel_feet_global);
            pub_v34(trotting_force_feet_global_pub_, tr.force_feet_global);
            pub_v34(trotting_force_feet_body_pub_, tr.force_feet_body);

            pub_v12(joint_q_goal_pub_, tr.q_goal);
            pub_v12(joint_qd_goal_pub_, tr.qd_goal);
            pub_v12(joint_tau_cmd_pub_, tr.tau_cmd);
            pub_v12(joint_q_state_pub_, tr.q_state);
            pub_v12(joint_qd_state_pub_, tr.qd_state);
            pub_v12(joint_tau_state_pub_, tr.tau_state);
            pub_v12(joint_q_error_pub_, tr.q_error);
            pub_v12(joint_qd_error_pub_, tr.qd_error);
            pub_v12(joint_tau_error_pub_, tr.tau_error);

            pub_v4(trotting_phase_pub_, tr.phase);
            pub_v4(trotting_contact_pub_, tr.contact);
            pub_scalar(trotting_yaw_cmd_pub_, tr.yaw_cmd);
            pub_scalar(trotting_yaw_est_pub_, tr.yaw_est);
            pub_scalar(trotting_yaw_error_pub_, tr.yaw_error);
            pub_scalar(trotting_dyaw_cmd_pub_, tr.d_yaw_cmd);
            pub_scalar(trotting_dyaw_est_pub_, tr.d_yaw_est);
            pub_scalar(trotting_wave_status_pub_, static_cast<double>(tr.wave_status));
        }
    }

    std::shared_ptr<FSMState> UnitreeGuideController::getNextState(const FSMStateName stateName) const
    {
        switch (stateName)
        {
        case FSMStateName::INVALID:
            return state_list_.invalid;
        case FSMStateName::PASSIVE:
            return state_list_.passive;
        case FSMStateName::FIXEDDOWN:
            return state_list_.fixedDown;
        case FSMStateName::FIXEDSTAND:
            return state_list_.fixedStand;
        case FSMStateName::FREESTAND:
            return state_list_.freeStand;
        case FSMStateName::TROTTING:
            return state_list_.trotting;
        case FSMStateName::SWINGTEST:
            return state_list_.swingTest;
        case FSMStateName::BALANCETEST:
            return state_list_.balanceTest;
        default:
            return state_list_.invalid;
        }
    }

}

// 插件声明，将其注册为ROS2控制器接口的实现，使得控制器管理器能够加载和使用它。
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(unitree_guide_controller::UnitreeGuideController, controller_interface::ControllerInterface);
