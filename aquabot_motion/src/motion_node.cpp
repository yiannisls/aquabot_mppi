#include <chrono>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp" // Required for Point #2
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class AquabotController : public rclcpp::Node {
public:
  AquabotController() : Node("aquabot_controller") {
    // 1. Subscribe to Odometry (Topic name matched to your system)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10, std::bind(&AquabotController::odom_callback, this, std::placeholders::_1));

    // 2. Subscribe to Plan (List of waypoints)
    plan_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/plan", 10, std::bind(&AquabotController::plan_callback, this, std::placeholders::_1));

    // 3. Publish on the 4 control topics under /aquabot/thrusters (Exact names from your system)
    pub_fl_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/front_left/thrust", 10);
    pub_fr_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/front_right/thrust", 10);
    pub_rl_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/rear_left/thrust", 10);
    pub_rr_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/rear_right/thrust", 10);
  }

private:
  // Callback for Plan (Point #2)
  void plan_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received %zu waypoints from /plan", msg->poses.size());
  }

  // Callback for Odometry (Point #1)
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Current X: %f", msg->pose.pose.position.x);
    
    // Point #3: Publishing a value to show it is working
    auto cmd = std_msgs::msg::Float64();
    cmd.data = 50.0; // Increased power so you can see movement
    pub_fl_->publish(cmd);
    pub_fr_->publish(cmd);
    pub_rl_->publish(cmd);
    pub_rr_->publish(cmd);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr plan_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fl_, pub_fr_, pub_rl_, pub_rr_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AquabotController>());
  rclcpp::shutdown();
  return 0;
}
