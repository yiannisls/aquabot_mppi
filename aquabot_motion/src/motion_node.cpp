#include <chrono>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <random>

using std::placeholders::_1;
using namespace std::chrono_literals;

class AquabotController : public rclcpp::Node {
public:
  AquabotController() : Node("aquabot_controller") {

    this->declare_parameter("dt", 0.1);
    this->declare_parameter("horizon", 30); 
    this->declare_parameter("lambda", 50.0);

    dt_ = this->get_parameter("dt").as_double();
    horizon_ = this->get_parameter("horizon").as_int();
    lambda_ = this->get_parameter("lambda").as_double();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/aquabot/odom", 10, std::bind(&AquabotController::odom_callback, this, std::placeholders::_1));

    plan_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
      "/aquabot/plan", 10, std::bind(&AquabotController::plan_callback, this, std::placeholders::_1));

    pub_fl_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/left/thrust", 10);
    pub_fr_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/right/thrust", 10);
    pub_al_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/left/cmd_pos", 10);
    pub_ar_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/right/cmd_pos", 10);

    pub_optimal_path_ = this->create_publisher<nav_msgs::msg::Path>("/aquabot/mppi/optimal_path", 10);
    pub_current_state_rollouts_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/aquabot/mppi/rollouts", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt_ * 1000.0)), std::bind(&AquabotController::control_loop, this));
    
    has_odom_ = false;
    plan_received_ = false;
    current_state_ = Eigen::VectorXd::Zero(6);
    goal_pose_ = Eigen::VectorXd::Zero(3);
    U_ = Eigen::MatrixXd::Zero(4, horizon_);
  }

private:
  void plan_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received an empty plan!");
      return;
    }
    current_plan_ = *msg;
    plan_received_ = true;

    auto end_pose = current_plan_.poses.back().pose;
    RCLCPP_INFO(this->get_logger(), "New Plan Received! Discretized Waypoints: %zu | End Goal: X: %.2f, Y: %.2f", 
                current_plan_.poses.size(), end_pose.position.x, end_pose.position.y);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_state_(0) = msg->pose.pose.position.x;
    current_state_(1) = msg->pose.pose.position.y;

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_state_(2) = yaw; 
    current_state_(3) = msg->twist.twist.linear.x; 
    current_state_(4) = msg->twist.twist.linear.y; 
    current_state_(5) = msg->twist.twist.angular.z; 

    has_odom_ = true;
  }

  Eigen::VectorXd calculate_dynamics(const Eigen::VectorXd& state, const Eigen::VectorXd& control) {
    double yaw = state(2);
    double u = state(3);
    double v = state(4);
    double r = state(5);

    double F_L = control(0) * 5000.0;
    double F_R = control(1) * 5000.0;
    double alpha_L = control(2);
    double alpha_R = control(3);

    double tau_X = (F_L * std::cos(alpha_L)) + (F_R * std::cos(alpha_R));
    double tau_Y = (F_L * std::sin(alpha_L)) + (F_R * std::sin(alpha_R));
    double tau_N = (W_ / 2.0) * ((F_R * std::cos(alpha_R)) - (F_L * std::cos(alpha_L))) 
                 - (L_ / 2.0) * ((F_L * std::sin(alpha_L)) + (F_R * std::sin(alpha_R)));

    double drag_u = (d_u_ * u) + (d_u_quad_ * u * std::abs(u));
    double drag_v = (d_v_ * v) + (d_v_quad_ * v * std::abs(v));
    double drag_r = (d_r_ * r) + (d_r_quad_ * r * std::abs(r));

    double u_dot = (tau_X - drag_u) / m_u_;
    double v_dot = (tau_Y - drag_v) / m_v_;
    double r_dot = (tau_N - drag_r) / m_r_;

    double x_dot = (u * std::cos(yaw)) - (v * std::sin(yaw));
    double y_dot = (u * std::sin(yaw)) + (v * std::cos(yaw));
    double yaw_dot = r;

    Eigen::VectorXd state_dot(6);
    state_dot << x_dot, y_dot, yaw_dot, u_dot, v_dot, r_dot;
    return state_dot;
  }
  
  Eigen::VectorXd predict_state(const Eigen::VectorXd& state, const Eigen::VectorXd& control) {
    Eigen::VectorXd k1 = calculate_dynamics(state, control);
    Eigen::VectorXd k2 = calculate_dynamics(state + (0.5 * dt_ * k1), control);
    Eigen::VectorXd k3 = calculate_dynamics(state + (0.5 * dt_ * k2), control);
    Eigen::VectorXd k4 = calculate_dynamics(state + (dt_ * k3), control);

    Eigen::VectorXd next_state = state + (dt_ / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    next_state(2) = std::atan2(std::sin(next_state(2)), std::cos(next_state(2)));

    next_state(3) = std::clamp(next_state(3), -10.0, 10.0); 
    next_state(4) = std::clamp(next_state(4), -4.0, 4.0);   
    next_state(5) = std::clamp(next_state(5), -2.0, 2.0);   

    return next_state;
  }

  double cost_function(const Eigen::VectorXd& state, const Eigen::VectorXd& goal) {
    double dx = goal(0) - state(0);
    double dy = goal(1) - state(1);
    double dist = std::sqrt((dx * dx) + (dy * dy));

    double yaw_desired = goal(2);
    double yaw_current = state(2);
    double yaw_error = std::atan2(std::sin(yaw_desired - yaw_current), 
                                std::cos(yaw_desired - yaw_current));
                                
    if (std::abs(yaw_error) < theta_deadband_) {
        yaw_error = 0.0;
    }                            
    // FIX: Add a Yaw Rate Damping penalty. 
    // This physically prevents the boat from oscillating in figure-8s!
    double yaw_rate_penalty = 5.0 * std::abs(state(5));

    // FIX: Rebalance the weights to be less frantic
    return (8.0 * dist) + (15.0 * std::abs(yaw_error)) + (yaw_rate_penalty);
  }

  void control_loop() {
    if(!plan_received_ || !has_odom_ || current_plan_.poses.empty()){ 
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for Odometry and Path Plan...");
      return;
    }

    double boat_x = current_state_(0);
    double boat_y = current_state_(1);

    // --- YOUR PURE PURSUIT DYNAMIC LOOKAHEAD ---
    double lookahead_distance = 5.0; // Distance in meters to look ahead
    auto target_pose = current_plan_.poses.back().pose; 

    // 1. Find the closest point on the path to the boat
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < current_plan_.poses.size(); ++i) {
      double px = current_plan_.poses[i].pose.position.x;
      double py = current_plan_.poses[i].pose.position.y;
      double dist = std::hypot(px - boat_x, py - boat_y);
      if (dist < min_dist) {
        min_dist = dist;
        closest_idx = i;
      }
    }

    // 2. Search forward from the closest point to find the lookahead target
    for (size_t i = closest_idx; i < current_plan_.poses.size(); ++i) {
      double px = current_plan_.poses[i].pose.position.x;
      double py = current_plan_.poses[i].pose.position.y;
      double dist = std::hypot(px - boat_x, py - boat_y);
      if (dist >= lookahead_distance) {
        target_pose = current_plan_.poses[i].pose;
        break;
      }
    }

    // 3. Set the MPPI goal coordinates
    goal_pose_(0) = target_pose.position.x;
    goal_pose_(1) = target_pose.position.y;
    
    // 4. THE PURE PURSUIT HEADING (Your original logic):
    // Calculate the angle from the boat directly to the lookahead point
    goal_pose_(2) = std::atan2(target_pose.position.y - boat_y, target_pose.position.x - boat_x);
    // --------------------------------------------

    double final_x = current_plan_.poses.back().pose.position.x;
    double final_y = current_plan_.poses.back().pose.position.y;
    double dist_to_end = std::hypot(final_x - boat_x, final_y - boat_y);

    if (dist_to_end < 2.0) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Goal Reached! Stopping.");
      
      std_msgs::msg::Float64 zero_msg;
      zero_msg.data = 0.0;
      pub_fl_->publish(zero_msg);
      pub_fr_->publish(zero_msg);
      pub_al_->publish(zero_msg);
      pub_ar_->publish(zero_msg);
      
      plan_received_ = false; 
      U_.setZero();
      return;
    }
    
    std::vector<double> rollout_costs(K_, 0.0);
    std::vector<Eigen::MatrixXd> rollout_noises(K_, Eigen::MatrixXd::Zero(4, horizon_));

    std::vector<visualization_msgs::msg::Marker> temp_markers;
    std::vector<double> temp_costs;

    for(int k=0; k<K_; k++){
      
      Eigen::VectorXd current_predicted_state_ = current_state_;
      double total_cost_ = 0.0;

      Eigen::VectorXd prev_noise = Eigen::VectorXd::Zero(4);
      double alpha = 0.6; 

      visualization_msgs::msg::Marker rollout_marker;
      bool visualize_this_rollout = (k % 10 == 0); 

      if (visualize_this_rollout) {
        rollout_marker.header.frame_id = "world"; 
        rollout_marker.header.stamp = this->now();
        rollout_marker.ns = "mppi_rollouts";
        rollout_marker.id = k;
        rollout_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        rollout_marker.action = visualization_msgs::msg::Marker::ADD;
        rollout_marker.scale.x = 0.20; 
      }
      
      for(int h=0; h<horizon_; h++){
        
        auto u_nominal = U_.col(h); 
        
        Eigen::VectorXd raw_noise(4);
        raw_noise(0) = noise_thrust_(gen_); 
        raw_noise(1) = noise_thrust_(gen_);
        raw_noise(2) = noise_angle_(gen_);  
        raw_noise(3) = noise_angle_(gen_);

        Eigen::VectorXd correlated_noise = (alpha * prev_noise) + ((1.0 - alpha) * raw_noise);
        prev_noise = correlated_noise; 

        Eigen::VectorXd noisy_control = u_nominal + correlated_noise;

        noisy_control(0) = std::clamp(noisy_control(0), -0.05, 0.20);
        noisy_control(1) = std::clamp(noisy_control(1), -0.05, 0.20);
        // FIX: Widened steering clamps to M_PI/8.0 so the boat can maneuver!
        noisy_control(2) = std::clamp(noisy_control(2), -M_PI/4.0, M_PI/4.0);
        noisy_control(3) = std::clamp(noisy_control(3), -M_PI/4.0, M_PI/4.0);

        rollout_noises[k].col(h) = noisy_control - u_nominal;

        current_predicted_state_ = predict_state(current_predicted_state_, noisy_control);

        // FIX: Passing the goal_pose_ carrot to the cost function
        double state_cost = cost_function(current_predicted_state_, goal_pose_);

        double control_penalty = 0.0;
        control_penalty += (u_nominal(0) * rollout_noises[k](0, h)) / var_thrust_;
        control_penalty += (u_nominal(1) * rollout_noises[k](1, h)) / var_thrust_;
        control_penalty += (u_nominal(2) * rollout_noises[k](2, h)) / var_angle_;
        control_penalty += (u_nominal(3) * rollout_noises[k](3, h)) / var_angle_;

        total_cost_ += state_cost + (lambda_ * control_penalty);
                
        if (visualize_this_rollout) {
            geometry_msgs::msg::Point p;
            p.x = current_predicted_state_(0);
            p.y = current_predicted_state_(1);
            p.z = 0.0;
            rollout_marker.points.push_back(p);
        }
      }

      if (visualize_this_rollout) {
          temp_markers.push_back(rollout_marker);
          temp_costs.push_back(total_cost_);
      }
      rollout_costs[k] = total_cost_;
    }

    double min_cost_ = *std::min_element(rollout_costs.begin(), rollout_costs.end());
    double max_cost_ = *std::max_element(rollout_costs.begin(), rollout_costs.end());
    double cost_range = std::max(max_cost_ - min_cost_, 1e-6); 

    visualization_msgs::msg::MarkerArray rollout_markers;

    for (size_t i = 0; i < temp_markers.size(); ++i) {
        double normalized_cost = (temp_costs[i] - min_cost_) / cost_range;
        normalized_cost = std::clamp(normalized_cost, 0.0, 1.0);

        temp_markers[i].color.r = normalized_cost; 
        temp_markers[i].color.g = 1.0 - normalized_cost; 
        temp_markers[i].color.b = 0.0;
        temp_markers[i].color.a = 0.5; 

        rollout_markers.markers.push_back(temp_markers[i]);
    }
    pub_current_state_rollouts_->publish(rollout_markers);

    double sum_weights_ = 0.0;
    std::vector<double> weights_(K_, 0.0);

    for (int k = 0; k < K_; k++) {
        weights_[k] = std::exp(-(rollout_costs[k] - min_cost_) / lambda_);
        sum_weights_ += weights_[k];
    }

    for (int k = 0; k < K_; k++) {
        weights_[k] /= sum_weights_;
        U_ += update_gain_ * weights_[k] * rollout_noises[k];
    }

    for (int h = 0; h < horizon_; h++) {
        U_(0, h) = std::clamp(U_(0, h), -0.05, 0.20);
        U_(1, h) = std::clamp(U_(1, h), -0.05, 0.20);
        // FIX: Match the widened steering clamps
        U_(2, h) = std::clamp(U_(2, h), -M_PI/4.0, M_PI/4.0);
        U_(3, h) = std::clamp(U_(3, h), -M_PI/4.0, M_PI/4.0);
    }

    Eigen::VectorXd cmd = U_.col(0);

    // --- FIX 2: SEPARATED RATE LIMITERS ---
    double max_thrust_delta = 0.05; // Thrust can change relatively quickly
    double max_angle_delta = 0.08;  // Steering turns smoothly to prevent whiplash

    // 1. Apply Thrust Rate Limits
    for(int i = 0; i < 2; i++) {
        double diff = cmd(i) - prev_published_cmd_(i);
        cmd(i) = prev_published_cmd_(i) + std::clamp(diff, -max_thrust_delta, max_thrust_delta);
    }
    // 2. Apply Steering Rate Limits 
    for(int i = 2; i < 4; i++) {
        double diff = cmd(i) - prev_published_cmd_(i);
        cmd(i) = prev_published_cmd_(i) + std::clamp(diff, -max_angle_delta, max_angle_delta);
    }
    prev_published_cmd_ = cmd;

    std_msgs::msg::Float64 msg_fl, msg_fr, msg_al, msg_ar;
    msg_fl.data = cmd(0) * 5000.0; 
    msg_fr.data = cmd(1) * 5000.0; 
    msg_al.data = cmd(2);
    msg_ar.data = cmd(3);

    pub_fl_->publish(msg_fl);
    pub_fr_->publish(msg_fr);
    pub_al_->publish(msg_al);
    pub_ar_->publish(msg_ar);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
      "Commanding Thrusters -> L: %.2f N, R: %.2f N | Angles -> L: %.2f rad, R: %.2f rad", msg_fl.data, msg_fr.data, msg_al.data, msg_ar.data);

    nav_msgs::msg::Path optimal_path;
    optimal_path.header.frame_id = "world";
    optimal_path.header.stamp = this->now();
    Eigen::VectorXd sim_state = current_state_;
    for (int h = 0; h < horizon_; h++) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = optimal_path.header;
        pose.pose.position.x = sim_state(0);
        pose.pose.position.y = sim_state(1);
        optimal_path.poses.push_back(pose);
        sim_state = predict_state(sim_state, U_.col(h));
    }
    pub_optimal_path_->publish(optimal_path);

    for(int h=0; h<horizon_-1; h++){
      U_.col(h) = U_.col(h+1);
    }     
    U_.col(horizon_ - 1).setZero();  
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fl_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fr_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_al_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ar_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_optimal_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_current_state_rollouts_;

  rclcpp::TimerBase::SharedPtr timer_;
  bool has_odom_;
  bool plan_received_;
  
  Eigen::VectorXd current_state_;
  Eigen::VectorXd goal_pose_;
  
  nav_msgs::msg::Path current_plan_;

  double dt_;
  int horizon_;
  int K_ = 1000; 

  Eigen::MatrixXd U_;
  double lambda_;

  Eigen::VectorXd prev_published_cmd_ = Eigen::VectorXd::Zero(4);

  const double L_ = 6.0; 
  const double W_ = 1.2; 
  const double m_u_ = 1000.0; 
  const double m_v_ = 1000.0; 
  const double m_r_ = 446.0;  
  const double d_u_ = 182.0;  
  const double d_v_ = 183.0;  
  const double d_r_ = 1199.0; 
  const double d_u_quad_ = 224.0; 
  const double d_v_quad_ = 149.0; 
  const double d_r_quad_ = 979.0; 
  const double update_gain_ = 0.85;       
  const double theta_deadband_ = 0.02;    

  const double std_thrust_ = 0.4;
  const double std_angle_ = 0.4;

  const double var_thrust_ = std_thrust_ * std_thrust_;
  const double var_angle_ = std_angle_ * std_angle_;

  std::mt19937 gen_{std::random_device{}()};
  std::normal_distribution<double> noise_thrust_{0.0, std_thrust_}; 
  std::normal_distribution<double> noise_angle_{0.0, std_angle_};  
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AquabotController>());
  rclcpp::shutdown();
  return 0;
}