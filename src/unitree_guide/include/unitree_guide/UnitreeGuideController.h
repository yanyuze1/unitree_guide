/**
 * 控制器头文件，定义了 UnitreeGuideController 类，负责实现四足机器人控制器的核心功能，包括接口配置、状态机实现和控制算法等。
 */

#ifndef QUADRUPEDCONTROLLER_H
#define QUADRUPEDCONTROLLER_H

#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/string.hpp>
#include <FSM/FSMState.h>
#include <FSM/StatePassive.h>
#include <FSM/StateFixedDown.h>
#include <common/enumClass.h>

#include <control/CtrlComponent.h>
#include <FSM/StateBalanceTest.h>
#include <FSM/StateFixedStand.h>
#include <FSM/StateFreeStand.h>
#include <FSM/StateSwingTest.h>
#include <FSM/StateTrotting.h>
#include <array>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <controller_interface/controller_interface.hpp>
#include <control_input_msgs/msg/inputs.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>




namespace unitree_guide_controller {
    struct FSMStateList {
        std::shared_ptr<FSMState> invalid;
        std::shared_ptr<StatePassive> passive;
        std::shared_ptr<StateFixedDown> fixedDown;
        std::shared_ptr<StateFixedStand> fixedStand;
        std::shared_ptr<StateFreeStand> freeStand;
        std::shared_ptr<StateTrotting> trotting;

        std::shared_ptr<StateSwingTest> swingTest;
        std::shared_ptr<StateBalanceTest> balanceTest;
    };

    class UnitreeGuideController final : public controller_interface::ControllerInterface {
    public:
        UnitreeGuideController() = default;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        controller_interface::CallbackReturn on_init() override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

        CtrlComponent ctrl_component_;
        CtrlInterfaces ctrl_interfaces_;

    protected:
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        std::string imu_name_;
        std::string base_name_;
        std::string command_prefix_;
        std::vector<std::string> imu_interface_types_;
        std::vector<std::string> feet_names_;

        // FR FL RR RL
        std::vector<double> stand_pos_ = {
            0.0, 0.67, -1.3,
            0.0, 0.67, -1.3,
            0.0, 0.67, -1.3,
            0.0, 0.67, -1.3
        };

        std::vector<double> down_pos_ = {
            0.0, 1.3, -2.4,
            0.0, 1.3, -2.4,
            0.0, 1.3, -2.4,
            0.0, 1.3, -2.4
        };

        double stand_kp_ = 80.0;
        double stand_kd_ = 3.5;

        rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr control_input_subscription_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> > *>
        command_interface_map_ = {
            {"effort", &ctrl_interfaces_.joint_torque_command_interface_},
            {"position", &ctrl_interfaces_.joint_position_command_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_command_interface_},
            {"kp", &ctrl_interfaces_.joint_kp_command_interface_},
            {"kd", &ctrl_interfaces_.joint_kd_command_interface_}
        };

        FSMMode mode_ = FSMMode::NORMAL;
        std::string state_name_;
        FSMStateName next_state_name_ = FSMStateName::INVALID;
        FSMStateList state_list_;
        std::shared_ptr<FSMState> current_state_;
        std::shared_ptr<FSMState> next_state_;

        std::chrono::time_point<std::chrono::steady_clock> last_update_time_;
        double update_frequency_;

        std::shared_ptr<FSMState> getNextState(FSMStateName stateName) const;

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > *>
        state_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_state_interface_},
            {"effort", &ctrl_interfaces_.joint_effort_state_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_state_interface_}
        };
        bool enable_debug_topics_ = true;
        int debug_topic_decimation_ = 4;
        int debug_topic_counter_ = 0;

        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr balance_dd_pcd_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr balance_d_wbd_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr balance_bd_force_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr balance_bd_torque_pub_;
        std::array<rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr, 4> balance_force_pub_{};

        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr balance_solved_force_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr balance_solved_torque_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr balance_force_error_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr balance_torque_error_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr balance_normal_force_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr balance_min_constraint_margin_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr balance_contact_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr balance_rot_matrix_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr balance_feet_pos_2_body_pub_;


        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr estimator_position_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr estimator_velocity_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr estimator_gyro_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr estimator_gyro_global_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr estimator_acceleration_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr estimator_u_world_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr estimator_yaw_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr estimator_dyaw_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr estimator_contact_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr estimator_phase_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr estimator_pos_meas_residual_norm_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr estimator_vel_meas_residual_norm_pub_;

        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr trotting_v_cmd_body_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr trotting_vel_target_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr trotting_pos_body_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr trotting_vel_body_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr trotting_pcd_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr trotting_pos_error_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr trotting_vel_error_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr trotting_dd_pcd_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr trotting_rot_error_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr trotting_d_wbd_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr trotting_gyro_global_pub_;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr trotting_pos_feet_global_goal_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr trotting_vel_feet_global_goal_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr trotting_pos_feet_global_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr trotting_vel_feet_global_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr trotting_force_feet_global_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr trotting_force_feet_body_pub_;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_q_goal_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_qd_goal_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_tau_cmd_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_q_state_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_qd_state_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_tau_state_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_q_error_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_qd_error_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_tau_error_pub_;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr trotting_phase_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr trotting_contact_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr trotting_yaw_cmd_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr trotting_yaw_est_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr trotting_yaw_error_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr trotting_dyaw_cmd_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr trotting_dyaw_est_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr trotting_wave_status_pub_;


        void publishDebugTopics(const rclcpp::Time &time);

    };
}


#endif //QUADRUPEDCONTROLLER_H
