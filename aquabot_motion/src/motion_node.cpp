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
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>



using std::placeholders::_1;
using namespace std::chrono_literals;

class AquabotController : public rclcpp::Node {
public:
  AquabotController() : Node("aquabot_controller") {

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/aquabot/odom", 10, std::bind(&AquabotController::odom_callback, this, std::placeholders::_1));

    // plan_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
    //   "/aquabot/plan", 10, std::bind(&AquabotController::plan_callback, this, std::placeholders::_1));
    goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/aquabot/goal_pose", 10, std::bind(&AquabotController::goal_callback, this, std::placeholders::_1));

    pub_fl_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/left/thrust", 10);
    pub_fr_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/right/thrust", 10);
    pub_al_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/left/cmd_pos", 10);
    pub_ar_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/right/cmd_pos", 10);


    // Publishers for visualisation
    pub_optimal_path_ = this->create_publisher<nav_msgs::msg::Path>("/aquabot/mppi/optimal_path", 10);
    pub_current_state_rollouts_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/aquabot/mppi/rollouts", 10);

    //10 Hz timer
    timer_ = this->create_wall_timer(100ms, std::bind(&AquabotController::control_loop, this));
    
    // flags to not start computing
    has_odom_ = false;
    goal_received_ = false;
    current_state_ = Eigen::VectorXd::Zero(6);
    goal_pose_ = Eigen::VectorXd::Zero(3);
    U_ = Eigen::MatrixXd::Zero(4, horizon_);
  }

private:
// void plan_callback(const nav_msgs::msg::Path::SharedPtr msg) {
  //   if (msg->poses.empty()) {
  //     RCLCPP_WARN(this->get_logger(), "Received an empty plan!");
  //     return;
  //   }
  //   current_plan_ = *msg;
  //   goal_received_ = true;

  //   auto end_pose = current_plan_.poses.back().pose;
  //   RCLCPP_INFO(this->get_logger(), "New Plan Received! End Goal: X: %.2f, Y: %.2f", 
  //               end_pose.position.x, end_pose.position.y);
  // }
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_pose_(0) = msg->pose.position.x;
    goal_pose_(1) = msg->pose.position.y;
    
    tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    goal_pose_(2) = yaw;
    
    goal_received_ = true;
    // print on terminal the pose
    RCLCPP_INFO(this->get_logger(), "New Goal Received: X: %.2f, Y: %.2f, Yaw: %.2f", goal_pose_(0), goal_pose_(1), goal_pose_(2));
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  
  has_odom_ = true;

  // global position
  current_state_(0) = msg->pose.pose.position.x;
  current_state_(1) = msg->pose.pose.position.y;

  //  orientation (yaw)
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


  current_state_(3) = msg->twist.twist.linear.x; // surge (u)
  current_state_(4) = msg->twist.twist.linear.y; // sway (v)
  current_state_(5) = msg->twist.twist.angular.z; // yaw rate (r)

  has_odom_ = true;

  }

  Eigen::VectorXd predict_state(const Eigen::VectorXd& state, const Eigen::VectorXd& control) {
    double x = state(0);
    double y = state(1);
    double yaw = state(2);
    double u = state(3);
    double v = state(4);
    double r = state(5);

    
    // Convert normalized thrust [-1.0, 1.0] to Newtons (5000N max thrust from URDF)
    // Otherwise 1 Newton won't move a 1000kg boat!
    double F_L = control(0) * 5000.0;
    double F_R = control(1) * 5000.0;
    double alpha_L = control(2);
    double alpha_R = control(3);

    // tau = [X, Y, N]^T

    double tau_X = (F_L * std::cos(alpha_L)) + (F_R * std::cos(alpha_R));
    double tau_Y = (F_L * std::sin(alpha_L)) + (F_R * std::sin(alpha_R));
    double tau_N = (L_ / 2.0) * ((F_R * std::cos(alpha_R)) - (F_L * std::cos(alpha_L))) - (W_ / 2.0) * ((F_L * std::sin(alpha_L)) + (F_R * std::sin(alpha_R)));

    // calculate local acelerations
    // M*v_dot + D*v = tau)
    // Therefor, v_dot = M^-1(tau-D(v)). since M diagonal, we can divide by M.
    double u_dot = (tau_X - d_u_ * u) / m_u_;
    double v_dot = (tau_Y - d_v_ * v) / m_v_;
    double r_dot = (tau_N - d_r_ * r) / m_r_;

    // global velocities. explained by proff Kermorgrant
    double x_dot = (u * std::cos(yaw)) - (v * std::sin(yaw));
    double y_dot = (u * std::sin(yaw)) + (v * std::cos(yaw));
    double yaw_dot = r;

    // Euler Integration
    Eigen::VectorXd next_state(6);
    next_state(0) = x + (x_dot * dt_);
    next_state(1) = y + (y_dot * dt_);
    next_state(2) = yaw + (yaw_dot * dt_);
    next_state(3) = u + (u_dot * dt_);
    next_state(4) = v + (v_dot * dt_);
    next_state(5) = r + (r_dot * dt_);

    // normalize the new yaw angle to stay between -PI and PI
    // This is what the professor said to stay betwen the pi's
    next_state(2) = std::atan2(std::sin(next_state(2)), std::cos(next_state(2)));

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

    // BRAKING SYSTEM: Heavily penalize velocity if the AI predicts it will be close to the goal.
    // This forces the AI to choose rollouts where it fires reverse thrust to slow down!
    double v_penalty = 0.0;
    if (dist < 4.0) {
        v_penalty = 20.0 * (std::abs(state(3)) + std::abs(state(4))); // penalize surge and sway speed
    }

    // Final Weighted Cost
    double cost = (15.0 * dist) + (5.0 * std::abs(yaw_error)) + v_penalty;

    return cost;
  }

  void control_loop() {
    // current_state Initialized from odom;
    // if(!goal_received_ || !has_odom_ || current_plan_.poses.empty()){ 
    //   RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for Odometry and Plan...");
    if(!goal_received_ || !has_odom_){ 
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for Odometry and Goal...");
      return;
    }

    double boat_x = current_state_(0);
    double boat_y = current_state_(1);

    // // Find a lookahead point on the path to set as the current target goal
    // double lookahead_distance = 5.0; // Distance in meters to look ahead on the path
    // auto target_pose = current_plan_.poses.back().pose; // Default to the end of the path

    // // 1. Find the closest point on the path
    // size_t closest_idx = 0;
    // double min_dist = std::hypot(current_plan_.poses[0].pose.position.x - boat_x,
    //                              current_plan_.poses[0].pose.position.y - boat_y);
    // for (size_t i = 1; i < current_plan_.poses.size(); ++i) {
    //   double px = current_plan_.poses[i].pose.position.x;
    //   double py = current_plan_.poses[i].pose.position.y;
    //   double dist = std::hypot(px - boat_x, py - boat_y);
    //   if (dist < min_dist) {
    //     min_dist = dist;
    //     closest_idx = i;
    //   }
    // }

    // // 2. From the closest point, find the lookahead target
    // for (size_t i = closest_idx; i < current_plan_.poses.size(); ++i) {
    //   double px = current_plan_.poses[i].pose.position.x;
    //   double py = current_plan_.poses[i].pose.position.y;
    //   double dist = std::hypot(px - boat_x, py - boat_y);
    //   if (dist >= lookahead_distance) {
    //     target_pose = current_plan_.poses[i].pose;
    //     break;
    //   }
    // }

    // // Update the dynamic goal pose for the MPPI cost function
    // goal_pose_(0) = target_pose.position.x;
    // goal_pose_(1) = target_pose.position.y;
    
    // tf2::Quaternion q(
    //   target_pose.orientation.x,
    //   target_pose.orientation.y,
    //   target_pose.orientation.z,
    //   target_pose.orientation.w
    // );
    
    // if (q.length() > 0.5) {
    //   tf2::Matrix3x3 m(q);
    //   double roll, pitch, yaw;
    //   m.getRPY(roll, pitch, yaw);
    //   goal_pose_(2) = yaw;
    // } else {
    //   // Fallback: Calculate heading pointing from the boat towards the lookahead point
    //   goal_pose_(2) = std::atan2(target_pose.position.y - boat_y, target_pose.position.x - boat_x);
    // }

    // Stop Condition
    // double final_x = current_plan_.poses.back().pose.position.x;
    // double final_y = current_plan_.poses.back().pose.position.y;
    // double dist_to_end = std::hypot(final_x - boat_x, final_y - boat_y);


    double dist_to_end = std::hypot(goal_pose_(0) - boat_x, goal_pose_(1) - boat_y);
    if (dist_to_end < 2.0) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Goal Reached! Stopping.");
      
      std_msgs::msg::Float64 zero_msg;
      zero_msg.data = 0.0;
      pub_fl_->publish(zero_msg);
      pub_fr_->publish(zero_msg);
      pub_al_->publish(zero_msg);
      pub_ar_->publish(zero_msg);
      
      goal_received_ = false;
      U_.setZero(); // Wipe out old commands
      return;
    }

    
    std::vector<double> rollout_costs(K_, 0.0);
    std::vector<Eigen::MatrixXd> rollout_noises(K_, Eigen::MatrixXd::Zero(4, horizon_));

    std::vector<visualization_msgs::msg::Marker> temp_markers;
    std::vector<double> temp_costs;

    for(int k=0; k<K_; k++){
      Eigen::VectorXd current_predicted_state_ = current_state_;
      double total_cost_ = 0.0;
      
      // Create a marker for this specific rollout
      visualization_msgs::msg::Marker rollout_marker;
      bool visualize_this_rollout = (k % 10 == 0); // only visualizing 1 out of 10 rollouts

      if (visualize_this_rollout) {
        rollout_marker.header.frame_id = "world"; // global frame odom
        rollout_marker.header.stamp = this->now();
        rollout_marker.ns = "mppi_rollouts";
        rollout_marker.id = k;
        rollout_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        rollout_marker.action = visualization_msgs::msg::Marker::ADD;
        rollout_marker.scale.x = 0.02; // Thin lines
      }
      

      for(int h=0; h<horizon_; h++){
        
        auto u_nominal = U_.col(h); // current nominal u
        
        //ADD NOISE
        Eigen::VectorXd noise = Eigen::VectorXd::Random(4); // values between -1.0 and 1.0
        noise(0) *= 0.5; // thrust noise variance
        noise(1) *= 0.5;
        noise(2) *= 0.2; // angle noise variance
        noise(3) *= 0.2;

        Eigen::VectorXd noisy_control = u_nominal + noise;

        // threshold to physical limits
        noisy_control(0) = std::clamp(noisy_control(0), -1.0, 1.0);
        noisy_control(1) = std::clamp(noisy_control(1), -1.0, 1.0);
        noisy_control(2) = std::clamp(noisy_control(2), -M_PI/4.0, M_PI/4.0);
        noisy_control(3) = std::clamp(noisy_control(3), -M_PI/4.0, M_PI/4.0);

        // store the ACTUAL applied noise for the weight update
        // In case it was clamped (0.8 + 0.5 nosie = 1.3, 
        // but clamped to 1.0 therefore noise will be 0.2)
        rollout_noises[k].col(h) = noisy_control - u_nominal;

        // PREDICT it
        current_predicted_state_ = predict_state(current_predicted_state_, noisy_control);

        // COST it
        total_cost_ += cost_function(current_predicted_state_, goal_pose_);
        
        // STORE points for visualization
        if (visualize_this_rollout) {
            geometry_msgs::msg::Point p;
            p.x = current_predicted_state_(0);
            p.y = current_predicted_state_(1);
            p.z = 0.0;
            rollout_marker.points.push_back(p);
        }
      }

      // Save the marker and its specific cost for the color pass
      if (visualize_this_rollout) {
          temp_markers.push_back(rollout_marker);
          temp_costs.push_back(total_cost_);
      }
      rollout_costs[k] = total_cost_;
    }
    // --- COLOR CODING PASS ---
    // First, find the bounds of our costs to create the gradient
    double min_cost_ = *std::min_element(rollout_costs.begin(), rollout_costs.end());
    double max_cost_ = *std::max_element(rollout_costs.begin(), rollout_costs.end());
    double cost_range = std::max(max_cost_ - min_cost_, 1e-6); // Prevent divide by zero

    visualization_msgs::msg::MarkerArray rollout_markers;

    for (size_t i = 0; i < temp_markers.size(); ++i) {
        // Normalize the cost between 0.0 (Best) and 1.0 (Worst)
        double normalized_cost = (temp_costs[i] - min_cost_) / cost_range;
        normalized_cost = std::clamp(normalized_cost, 0.0, 1.0);

        // Color Math: Green to Red
        // As normalized_cost gets higher (worse), Red goes up and Green goes down.
        temp_markers[i].color.r = normalized_cost; 
        temp_markers[i].color.g = 1.0 - normalized_cost; 
        temp_markers[i].color.b = 0.0;
        
        // Keep the alpha low so the overlapping green lines glow brightly
        temp_markers[i].color.a = 0.15; 

        rollout_markers.markers.push_back(temp_markers[i]);
    }

    pub_current_state_rollouts_->publish(rollout_markers);
    // -------------------------

    
    // SOFTMAX WEIGHTING to find the best path


    
    double sum_weights_ = 0.0;
    std::vector<double> weights_(K_, 0.0);

    for (int k = 0; k < K_; k++) {
        // Softmax Equation: e^(-(cost - min) / lambda)
        // Lower costs result in higher weights.
        weights_[k] = std::exp(-(rollout_costs[k] - min_cost_) / lambda_);
        sum_weights_ += weights_[k];
    }

    // Normalize the weights
    for (int k = 0; k < K_; k++) {
        weights_[k] /= sum_weights_;
        // multiply scalar weight to the whole matrix
        U_ += weights_[k] * rollout_noises[k];

    }

    // Clamp U_ to bounds so the nominal trajectory remains valid
    for (int h = 0; h < horizon_; h++) {
        U_(0, h) = std::clamp(U_(0, h), -1.0, 1.0);
        U_(1, h) = std::clamp(U_(1, h), -1.0, 1.0);
        U_(2, h) = std::clamp(U_(2, h), -M_PI/4.0, M_PI/4.0);
        U_(3, h) = std::clamp(U_(3, h), -M_PI/4.0, M_PI/4.0);
    }

    // FINAL STEP (finally)
    // publish and shift
    Eigen::VectorXd cmd = U_.col(0);

    std_msgs::msg::Float64 msg_fl, msg_fr, msg_al, msg_ar;
    msg_fl.data = cmd(0) * 5000.0; // Scale to real Newtons
    msg_fr.data = cmd(1) * 5000.0; // Scale to real Newtons
    msg_al.data = cmd(2);
    msg_ar.data = cmd(3);

    pub_fl_->publish(msg_fl);
    pub_fr_->publish(msg_fr);
    pub_al_->publish(msg_al);
    pub_ar_->publish(msg_ar);

    // Print the commands being published to the thrusters
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
      "Commanding Thrusters -> L: %.2f N, R: %.2f N | Angles -> L: %.2f rad, R: %.2f rad", msg_fl.data, msg_fr.data, msg_al.data, msg_ar.data);

    for(int h=0; h<horizon_-1; h++){
      U_.col(h) = U_.col(h+1);
    }     
    // Set the very last timestep to 0 so the boat knows to stop eventually
    U_.col(horizon_ - 1).setZero();  

  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  // rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fl_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fr_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_al_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ar_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_optimal_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_current_state_rollouts_;

  rclcpp::TimerBase::SharedPtr timer_;
  bool has_odom_;
  bool goal_received_;
  Eigen::VectorXd goal_pose_;
  Eigen::VectorXd current_state_;
  // nav_msgs::msg::Path current_plan_;

  // MPPI parameters
  double dt_ = 0.1; // 10 Hz
  int horizon_ = 60; // Increased to 6 seconds of foresight to stop overshoot!
  int K_ = 500; //rollouts

  // 4 rows (controls), horizon_ columns (future steps)
  Eigen::MatrixXd U_;

  // Temperature for softmax weighting
  double lambda_ = 50.0;
  // Physical parameters (from URDF)
  const double L_ = 6.0; // Distance scaled so L_/2 = 3.0m (Thrusters at X = -3.0)
  const double W_ = 1.2; // Distance scaled so W_/2 = 0.6m (Thrusters at Y = +/-0.6)
  const double m_u_ = 1000.0; // Surge mass
  const double m_v_ = 1000.0; // Sway mass
  const double m_r_ = 446.0;  // Yaw inertia (Izz)
  const double d_u_ = 182.0;  // Surge linear drag
  const double d_v_ = 183.0;  // Sway linear drag
  const double d_r_ = 1199.0; // Yaw linear drag
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AquabotController>());
  rclcpp::shutdown();
  return 0;
}
