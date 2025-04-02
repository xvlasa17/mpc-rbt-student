#include "mpc_rbt_solution/MotionControl.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionControlNode>());
    rclcpp::shutdown();
    return 0;
}
