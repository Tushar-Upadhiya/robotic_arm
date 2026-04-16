#include "kinematics_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

using namespace std::chrono_literals;

class ArmSolver : public rclcpp::Node {
public:
    ArmSolver() : Node("arm_solver") {
        this->declare_parameter("d1", 0.09025); 
        this->declare_parameter("a2", 0.175);   
        this->declare_parameter("a3", 0.15375); 
        this->declare_parameter("d4", 0.042);

        timer_ = this->create_wall_timer(
            1000ms, std::bind(&ArmSolver::timer_callback, this));
    }

    void calculate_fk(double q1, double q2, double q3, double q4) {
        double d1 = this->get_parameter("d1").as_double();
        double a2 = this->get_parameter("a2").as_double();
        double a3 = this->get_parameter("a3").as_double();
        double d4 = this->get_parameter("d4").as_double();

        Eigen::Matrix4d T01 = robo_math::dh_transform(q1, d1, 0, M_PI_2);
        Eigen::Matrix4d T12 = robo_math::dh_transform(q2, 0, a2, 0);
        Eigen::Matrix4d T23 = robo_math::dh_transform(q3, 0, a3, 0);
        Eigen::Matrix4d T34 = robo_math::dh_transform(q4, d4, 0, 0);

        Eigen::Matrix4d T_total = T01 * T12 * T23 * T34;

        auto pos = T_total.block<3,1>(0,3);
        RCLCPP_INFO(this->get_logger(), "End Effector at: X:%.3f Y:%.3f Z:%.3f", pos.x(), pos.y(), pos.z());
    }

private:
    void timer_callback() {
        calculate_fk(0.0, 0.0, 0.0, 0.0);
    }
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmSolver>());
  rclcpp::shutdown();
  return 0;
}