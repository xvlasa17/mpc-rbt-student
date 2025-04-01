#include "mpc_rbt_solution/Planning.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlanningNode>());
    rclcpp::shutdown();
    return 0;
}
