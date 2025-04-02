#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "../include/Localization.hpp"

LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"), 
    last_time_(this->get_clock()->now()) {

    // Odometry message initialization
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";
    // add code here

    // Subscriber for joint_states
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,std::bind(&LocalizationNode::jointCallback,this,std::placeholders::_1));

    // Publisher for odometry
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry",10);

    // tf_briadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {


    // ********
    // * Help *
    // ********
    
    //auto current_time = this->get_clock()->now();
    //double dt = current_time - last_time_;
    //last_time_ = current_time;
    double dt = 10;


    updateOdometry(msg.velocity[0], msg.velocity[1], dt);
    publishOdometry();
    publishTransform();

    RCLCPP_INFO(get_logger(), "v1: %f v2: %f", msg.velocity[0], msg.velocity[1]);
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    // add code here

    // ********
    // * Help *
    // ********
    
    double linear = (left_wheel_vel + right_wheel_vel)/2;
    double angular =  (right_wheel_vel - left_wheel_vel)/robot_config::HALF_DISTANCE_BETWEEN_WHEELS; //robot_config::HALF_DISTANCE_BETWEEN_WHEELS

    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);

    theta = std::atan2(std::sin(theta), std::cos(theta));

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    
}

void LocalizationNode::publishOdometry() {
    // add code here
    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
    // add code here
    
    
    // ********
    // * Help *
    // ********
    //tf_broadcaster_->sendTransform(t);
}
