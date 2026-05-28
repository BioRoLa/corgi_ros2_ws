#include "corgi_force_estimation/force_estimation.hpp"

namespace {
constexpr double kDt = 0.001;
}

class AdmittanceControlNode : public rclcpp::Node {
public:
    AdmittanceControlNode()
        : Node("admittance_control"),
          sim_(false),
          pos_err_hist_modules_(4, Eigen::MatrixXd::Zero(2, 2)),
          force_err_hist_modules_(4, Eigen::MatrixXd::Zero(2, 2))
    {
        RCLCPP_INFO(this->get_logger(), "Admittance Control Starts");

        this->get_parameter_or("use_sim_time", sim_, false);
        kinematics_ = std::make_unique<KinematicsHelper>(sim_);

        if (sim_) {
            RCLCPP_INFO(this->get_logger(), "Waiting for Webots clock...");
            while (rclcpp::ok()) {
                rclcpp::spin_some(this->get_node_base_interface());
                if (this->now().seconds() > 0.0) {
                    RCLCPP_INFO(this->get_logger(), "Clock synced! Sim Time: %.2f", this->now().seconds());
                    break;
                }
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Real hardware mode: using system wall clock.");
        }

        motor_state_sub_ = this->create_subscription<corgi_msgs::msg::MotorStateStamped>(
            "motor/state", 10,
            std::bind(&AdmittanceControlNode::motor_state_cb, this, std::placeholders::_1));

        force_state_sub_ = this->create_subscription<corgi_msgs::msg::ForceStateStamped>(
            "force/state", 10,
            std::bind(&AdmittanceControlNode::force_state_cb, this, std::placeholders::_1));

        imp_cmd_sub_ = this->create_subscription<corgi_msgs::msg::ImpedanceCmdStamped>(
            "impedance/command", 10,
            std::bind(&AdmittanceControlNode::imp_cmd_cb, this, std::placeholders::_1));

        motor_cmd_pub_ = this->create_publisher<corgi_msgs::msg::MotorCmdStamped>(
            "motor/command", 10);
    }

    void run() {
        rclcpp::Duration period(0, 1000000); // 1ms
        rclcpp::Time next_time = this->now();

        while (rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());

            std::vector<corgi_msgs::msg::MotorState*> motor_state_modules = {
                &motor_state_.module_a,
                &motor_state_.module_b,
                &motor_state_.module_c,
                &motor_state_.module_d
            };

            std::vector<corgi_msgs::msg::ForceState*> force_state_modules = {
                &force_state_.module_a,
                &force_state_.module_b,
                &force_state_.module_c,
                &force_state_.module_d
            };

            std::vector<corgi_msgs::msg::ImpedanceCmd*> imp_cmd_modules = {
                &imp_cmd_.module_a,
                &imp_cmd_.module_b,
                &imp_cmd_.module_c,
                &imp_cmd_.module_d
            };

            std::vector<corgi_msgs::msg::MotorCmd*> motor_cmd_modules = {
                &motor_cmd_.module_a,
                &motor_cmd_.module_b,
                &motor_cmd_.module_c,
                &motor_cmd_.module_d
            };

            Eigen::MatrixXd eta_cmd = Eigen::MatrixXd::Zero(2, 1);

            for (int i = 0; i < 4; i++) {
                eta_cmd = admittance_control(imp_cmd_modules[i], motor_state_modules[i], force_state_modules[i],
                                             pos_err_hist_modules_[i], force_err_hist_modules_[i]);

                motor_cmd_modules[i]->kp_r = 90.0;
                motor_cmd_modules[i]->kp_l = 90.0;
                motor_cmd_modules[i]->kd_r = 1.75;
                motor_cmd_modules[i]->kd_l = 1.75;

                motor_cmd_modules[i]->theta = eta_cmd(0, 0);
                motor_cmd_modules[i]->beta  = eta_cmd(1, 0);
            }

            motor_cmd_.header.stamp = this->now();
            motor_cmd_pub_->publish(motor_cmd_);

            next_time += period;
            if (!this->get_clock()->sleep_until(next_time)) {
                RCLCPP_WARN(this->get_logger(), "Sleep until failed!");
                break;
            }
        }
    }

private:
    void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg) {
        motor_state_ = *msg;
    }

    void force_state_cb(const corgi_msgs::msg::ForceStateStamped::SharedPtr msg) {
        force_state_ = *msg;
    }

    void imp_cmd_cb(const corgi_msgs::msg::ImpedanceCmdStamped::SharedPtr msg) {
        imp_cmd_ = *msg;
    }

    Eigen::MatrixXd admittance_control(const corgi_msgs::msg::ImpedanceCmd* imp_cmd_,
                                       const corgi_msgs::msg::MotorState* motor_state_,
                                       const corgi_msgs::msg::ForceState* force_state_,
                                       Eigen::MatrixXd& pos_err_hist, Eigen::MatrixXd& force_err_hist) {

        Eigen::MatrixXd eta_cmd(2, 1);
        eta_cmd << imp_cmd_->theta, imp_cmd_->beta;

        Eigen::MatrixXd pos_des(2, 1);
        Eigen::MatrixXd pos_cmd(2, 1);
        Eigen::MatrixXd pos_err(2, 1);
        Eigen::MatrixXd force_err(2, 1);
        int target_rim;

        LegModel& legmodel = kinematics_->get_leg_model();
        legmodel.contact_map(imp_cmd_->theta, imp_cmd_->beta);
        pos_des << legmodel.contact_p[0], legmodel.contact_p[1];
        target_rim = legmodel.rim;

        force_err << imp_cmd_->fx - force_state_->fx, imp_cmd_->fy - force_state_->fy;

        double T = kDt;

        Eigen::MatrixXd M(2, 2);
        Eigen::MatrixXd B(2, 2);
        Eigen::MatrixXd K(2, 2);

        M << imp_cmd_->mx, 0, 0, imp_cmd_->my;
        B << imp_cmd_->bx, 0, 0, imp_cmd_->by;
        K << imp_cmd_->kx, 0, 0, imp_cmd_->ky;

        Eigen::MatrixXd a0 =  4*M + 2*B*T + K*T*T;
        Eigen::MatrixXd a1 = -8*M + 2*K*T*T;
        Eigen::MatrixXd a2 =  4*M - 2*B*T + K*T*T;
        Eigen::MatrixXd b0 = Eigen::MatrixXd::Identity(2, 2) * T * T;
        Eigen::MatrixXd b1 = Eigen::MatrixXd::Identity(2, 2) * 2 * T * T;
        Eigen::MatrixXd b2 = Eigen::MatrixXd::Identity(2, 2) * T * T;

        if (!a0.allFinite() || std::abs(a0.determinant()) < 1e-9) {
            return eta_cmd;
        }

        Eigen::MatrixXd rhs = b0*force_err + b1*force_err_hist.col(0) + b2*force_err_hist.col(1)
                               - a1*pos_err_hist.col(0) - a2*pos_err_hist.col(1);
        pos_err = a0.ldlt().solve(rhs);
        if (!pos_err.allFinite()) {
            return eta_cmd;
        }

        pos_cmd = pos_des - pos_err;

        if (pos_cmd.array().isNaN().any()) { return eta_cmd; }

        legmodel.contact_map(motor_state_->theta, motor_state_->beta);
        pos_err_hist.col(1) = pos_err_hist.col(0);
        pos_err_hist.col(0) << pos_des(0, 0) - pos_cmd(0, 0), pos_des(1, 0) - pos_cmd(1, 0);
        force_err_hist.col(1) = force_err_hist.col(0);
        force_err_hist.col(0) = force_err;

        std::array<double, 2> eta = {eta_cmd(0, 0), eta_cmd(1, 0)};
        double contact_center[2] = {pos_cmd(0, 0), pos_cmd(1, 0)};

        switch (target_rim) {
        case 1:
            contact_center[0] = pos_cmd(0, 0);
            contact_center[1] = pos_cmd(1, 0) + legmodel.radius;
            eta = legmodel.inverse(contact_center, "U_l");
            break;
        case 2:
            contact_center[0] = pos_cmd(0, 0);
            contact_center[1] = pos_cmd(1, 0) + legmodel.radius;
            eta = legmodel.inverse(contact_center, "L_l");
            break;
        case 3:
            contact_center[0] = pos_cmd(0, 0);
            contact_center[1] = pos_cmd(1, 0) + legmodel.r;
            eta = legmodel.inverse(contact_center, "G");
            break;
        case 4:
            contact_center[0] = pos_cmd(0, 0);
            contact_center[1] = pos_cmd(1, 0) + legmodel.radius;
            eta = legmodel.inverse(contact_center, "L_r");
            break;
        case 5:
            contact_center[0] = pos_cmd(0, 0);
            contact_center[1] = pos_cmd(1, 0) + legmodel.radius;
            eta = legmodel.inverse(contact_center, "U_r");
            break;
        default:
            break;
        }

        eta_cmd << eta[0], eta[1];
        if (!eta_cmd.allFinite()) {
            eta_cmd(0, 0) = motor_state_->theta;
            eta_cmd(1, 0) = motor_state_->beta;
            return eta_cmd;
        }

        return eta_cmd;
    }

    // Kinematics helper
    std::unique_ptr<KinematicsHelper> kinematics_;

    // Messages
    corgi_msgs::msg::MotorStateStamped motor_state_;
    corgi_msgs::msg::ForceStateStamped force_state_;
    corgi_msgs::msg::ImpedanceCmdStamped imp_cmd_;
    corgi_msgs::msg::MotorCmdStamped motor_cmd_;

    // ROS2 interfaces
    rclcpp::Subscription<corgi_msgs::msg::MotorStateStamped>::SharedPtr motor_state_sub_;
    rclcpp::Subscription<corgi_msgs::msg::ForceStateStamped>::SharedPtr force_state_sub_;
    rclcpp::Subscription<corgi_msgs::msg::ImpedanceCmdStamped>::SharedPtr imp_cmd_sub_;
    rclcpp::Publisher<corgi_msgs::msg::MotorCmdStamped>::SharedPtr motor_cmd_pub_;

    // State variables
    bool sim_;
    std::vector<Eigen::MatrixXd> pos_err_hist_modules_;
    std::vector<Eigen::MatrixXd> force_err_hist_modules_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AdmittanceControlNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}