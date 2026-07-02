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
#include <omp.h>
#include <fstream>
#include <iomanip>
#include <ctime>

struct MPPIParams {
    int horizon;
    float dt;
    float L, W;
    float m_u, m_v, m_r;
    float d_u, d_v, d_r;
    float d_u_quad, d_v_quad, d_r_quad;
    float q_cross, q_along, q_theta;
    float F_max;

    // Added extra weights from CPU cost function to be used in CUDA kernel
    float speed_weight;
    float yaw_rate_weight;
    float thrust_weight;
    float steer_weight;
    float target_speed;
};

extern "C" void launch_mppi_cuda(
    float* h_initial_state, float* h_nominal_U, float* h_noise, 
    float* h_ref_path, float* h_costs, MPPIParams params, int K);

using std::placeholders::_1;
using namespace std::chrono_literals;

class AquabotController : public rclcpp::Node {
public:
  AquabotController() : Node("aquabot_controller") {

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/aquabot/odom", 10, std::bind(&AquabotController::odom_callback, this, std::placeholders::_1));

    plan_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
      "/aquabot/plan", 10, std::bind(&AquabotController::plan_callback, this, std::placeholders::_1));

    pub_fl_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/left/thrust", 10);
    pub_fr_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/right/thrust", 10);
    pub_al_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/left/cmd_pos", 10);
    pub_ar_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/right/cmd_pos", 10);

    pub_optimal_path_ = this->create_publisher<nav_msgs::msg::Path>("/aquabot/mppi/optimal_path", 1);
    pub_current_state_rollouts_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/aquabot/mppi/rollouts", 1);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt_ * 1000.0)), std::bind(&AquabotController::control_loop, this));
    
    has_odom_ = false;
    plan_received_ = false;
    current_state_ = Eigen::VectorXd::Zero(6);
    U_ = Eigen::MatrixXd::Zero(4, horizon_);

    rollout_costs_.assign(K_, 0.0);
    rollout_noises_.assign(K_, Eigen::MatrixXd::Zero(4, horizon_));

    // --- CSV logging for post-run analysis (path tracking, speed, effort) ---
    auto t = std::time(nullptr);
    char fname[128];
    std::strftime(fname, sizeof(fname), "/tmp/aquabot_run_%Y%m%d_%H%M%S.csv", std::localtime(&t));
    log_file_.open(fname);
    log_file_ << "time,x,y,yaw,speed,speed_error,cross_track_error,yaw_rate,"
                 "thrust_L,thrust_R,steer_L,steer_R,min_cost\n";
    RCLCPP_INFO(this->get_logger(), "Logging run data to %s", fname);
  }

  ~AquabotController() {
    if (log_file_.is_open()) log_file_.close();
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

    // Coriolis-centripetal coupling (3-DOF Fossen, diagonal mass matrix).
    // This is what makes a turning boat bleed surge into sway and vice-versa.
    // With m_u_ == m_v_ the yaw term (cor_r) is exactly zero.
    double cor_u =   m_v_ * v * r;            // sway x yaw -> surge
    double cor_v =  -m_u_ * u * r;            // surge x yaw -> sway
    double cor_r =  (m_u_ - m_v_) * u * v;   // surge x sway -> yaw  (0 while m_u==m_v)

    double u_dot = (tau_X - drag_u + cor_u) / m_u_;
    double v_dot = (tau_Y - drag_v + cor_v) / m_v_;
    double r_dot = (tau_N - drag_r + cor_r) / m_r_;

    double x_dot = (u * std::cos(yaw)) - (v * std::sin(yaw));
    double y_dot = (u * std::sin(yaw)) + (v * std::cos(yaw));
    double yaw_dot = r;

    Eigen::VectorXd state_dot(6);
    state_dot << x_dot, y_dot, yaw_dot, u_dot, v_dot, r_dot;
    return state_dot;
  }
  
  Eigen::VectorXd predict_state(const Eigen::VectorXd& state, const Eigen::VectorXd& control_rates) {
    // 1. Integrate the control rates to get the actual actuator commands
    double T_L_next = state(6) + (control_rates(0) * dt_);
    double T_R_next = state(7) + (control_rates(1) * dt_);
    double A_L_next = state(8) + (control_rates(2) * dt_);
    double A_R_next = state(9) + (control_rates(3) * dt_);

    // 2. Enforce the physical limits of the thrusters here, natively in the math
    T_L_next = std::clamp(T_L_next, min_thrusters_clamped, max_thrusters_clamped);
    T_R_next = std::clamp(T_R_next, min_thrusters_clamped, max_thrusters_clamped);
    A_L_next = std::clamp(A_L_next, min_rotation_clamped, max_rotation_clamped);
    A_R_next = std::clamp(A_R_next, min_rotation_clamped, max_rotation_clamped);

    Eigen::VectorXd absolute_controls(4);
    absolute_controls << T_L_next, T_R_next, A_L_next, A_R_next;

    // 3. Run your standard dynamics using the absolute controls
    Eigen::VectorXd k1 = calculate_dynamics(state.head(6), absolute_controls);
    Eigen::VectorXd k2 = calculate_dynamics(state.head(6) + (0.5 * dt_ * k1), absolute_controls);
    Eigen::VectorXd k3 = calculate_dynamics(state.head(6) + (0.5 * dt_ * k2), absolute_controls);
    Eigen::VectorXd k4 = calculate_dynamics(state.head(6) + (dt_ * k3), absolute_controls);

    Eigen::VectorXd next_state(10);
    next_state.head(6) = state.head(6) + (dt_ / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    next_state(2) = std::atan2(std::sin(next_state(2)), std::cos(next_state(2)));

    // Clamp velocities as you did before
    next_state(3) = std::clamp(next_state(3), -10.0, 10.0); 
    next_state(4) = std::clamp(next_state(4), -4.0, 4.0);   
    next_state(5) = std::clamp(next_state(5), -2.0, 2.0);   

    // Save the new actuator states
    next_state(6) = T_L_next;
    next_state(7) = T_R_next;
    next_state(8) = A_L_next;
    next_state(9) = A_R_next;

    return next_state;
  }

  double cost_function(const Eigen::VectorXd& state, const Eigen::VectorXd& ref_point, const Eigen::VectorXd& control) {
    double dx = ref_point(0) - state(0);
    double dy = ref_point(1) - state(1);
    double ref_yaw = ref_point(2);

    // Cross-track / along-track decomposition (mirror of the GPU kernel)
    double e_cross = -std::sin(ref_yaw)*dx + std::cos(ref_yaw)*dy;
    double e_along =  std::cos(ref_yaw)*dx + std::sin(ref_yaw)*dy;
    double ec = std::abs(e_cross);
    double cross_pen = (ec < 1.0) ? (ec*ec) : (2.0*ec - 1.0);
    double along_pen = e_along * e_along;

    // 2. Heading Error
    double yaw_desired = ref_point(2); 
    double yaw_current = state(2);
    double yaw_error = std::atan2(std::sin(yaw_desired - yaw_current), std::cos(yaw_desired - yaw_current));
    if (std::abs(yaw_error) < theta_deadband_) yaw_error = 0.0;
    double yaw_error_sq = yaw_error * yaw_error;


    // 3. Target Speed Critic (Smooth Cruising, NO BRAKE SLAMMING)
    double current_speed = std::hypot(state(3), state(4));
    // The boat must maintain the target_speed_ at all times, even in curves!
    double speed_penalty = speed_weight_ * std::pow(current_speed - target_speed_, 2);

    // 4. Spin-Out Prevention
    double yaw_rate_penalty = yaw_rate_weight_ * (state(5) * state(5)); 
    
    // 5. CRITICAL FIX: The Steering Penalty!
    double actuator_thrust_penalty = thrust_weight_ * (control(0)*control(0) + control(1)*control(1));
    // Force the AI to return the steering wheel to 0.0!
    double actuator_steer_penalty = steer_weight_ * (control(2)*control(2) + control(3)*control(3)); 


    return (q_cross_ * cross_pen) +
           (q_along_ * along_pen) +
           (yaw_weight_ * yaw_error_sq) + 
           speed_penalty + 
           yaw_rate_penalty +
           actuator_thrust_penalty + 
           actuator_steer_penalty;
  }

  void control_loop() {
    if(!plan_received_ || !has_odom_ || current_plan_.poses.size() < 2){ 
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for Odometry and Path Plan...");
      return;
    }

    // ---- TIMING HARNESS ----
    auto ms = [](auto a, auto b){ return std::chrono::duration<double,std::milli>(b-a).count(); };
    auto t_start = std::chrono::steady_clock::now();

    double boat_x = current_state_(0);
    double boat_y = current_state_(1);

    // --- 1. Find Closest Point ---
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

    // --- 2. Check if Goal Reached ---
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

    // --- 3. THE GHOST BOAT: Build the Reference Trajectory ---

    // The ghost carrot tracks the boat's speed (floored + small lead, capped at cruise).
    // NOTE: the SPEED CRITIC still targets the fixed target_speed_ = 1.3 (see params below),
    // so the carrot placement and the speed target stay consistent at cruise.
    double current_speed = std::hypot(current_state_(3), current_state_(4));

    const double ghost_floor   = 0.40;  // carrot never crawls below this
    const double accel_margin  = 0.15;  // keep the carrot slightly ahead so the boat accelerates
    double dynamic_ghost_speed = std::clamp(current_speed + accel_margin,
                                            ghost_floor, ghost_target_speed_);
    
    
    std::vector<double> cum_dist(current_plan_.poses.size(), 0.0);  
    
    for (size_t i = closest_idx + 1; i < current_plan_.poses.size(); ++i) {
      double dx = current_plan_.poses[i].pose.position.x - current_plan_.poses[i-1].pose.position.x;
      double dy = current_plan_.poses[i].pose.position.y - current_plan_.poses[i-1].pose.position.y;
      cum_dist[i] = cum_dist[i-1] + std::hypot(dx, dy);
    }

    std::vector<Eigen::VectorXd> ref_traj(horizon_, Eigen::VectorXd::Zero(3));
    
    double initial_offset = 0; // Tight, stable tracking distance

    for (int h = 0; h < horizon_; ++h) {
      // Use dynamic_ghost_speed instead of the hardcoded ghost_speed_
      double target_d = initial_offset + (h * dynamic_ghost_speed * dt_);
      
      size_t idx = closest_idx + 1;
      while (idx < current_plan_.poses.size() && cum_dist[idx] < target_d) {
        idx++;
      }
      
      if (idx >= current_plan_.poses.size()) {
        idx = current_plan_.poses.size() - 1;
        ref_traj[h](0) = current_plan_.poses[idx].pose.position.x;
        ref_traj[h](1) = current_plan_.poses[idx].pose.position.y;
        // Keep the tangent of the final approach
        if (idx > 0) {
            ref_traj[h](2) = std::atan2(current_plan_.poses[idx].pose.position.y - current_plan_.poses[idx-1].pose.position.y,
                                        current_plan_.poses[idx].pose.position.x - current_plan_.poses[idx-1].pose.position.x);
        }
      } else {
        double d1 = cum_dist[idx-1];
        double d2 = cum_dist[idx];
        double ratio = (target_d - d1) / std::max(d2 - d1, 1e-6);
        
        double x1 = current_plan_.poses[idx-1].pose.position.x;
        double y1 = current_plan_.poses[idx-1].pose.position.y;
        double x2 = current_plan_.poses[idx].pose.position.x;
        double y2 = current_plan_.poses[idx].pose.position.y;
        
        ref_traj[h](0) = x1 + ratio * (x2 - x1);
        ref_traj[h](1) = y1 + ratio * (y2 - y1);
        
        // TANGENT HEADING: Calculate the exact direction the path is going
        ref_traj[h](2) = std::atan2(y2 - y1, x2 - x1); 
      }
    }

    auto t_ref = std::chrono::steady_clock::now();

    // 4. MPPI Phase 

    // 1. Generate noise on CPU (common mode + small differential mode per channel)
    std::vector<float> h_noise(K_ * horizon_ * 4);

    // SPLIT THIS LOOP ACROSS ALL CPU CORES!
    #pragma omp parallel for
    for(int k=0; k<K_; k++){

      // thread-safe: each core gets its own generator and its own distributions
      unsigned seed = std::chrono::system_clock::now().time_since_epoch().count() + k;
      std::mt19937 local_gen(seed);

      std::normal_distribution<float> n_thrust_common(0.0f, std_thrust_);
      std::normal_distribution<float> n_thrust_diff  (0.0f, std_thrust_diff_);
      std::normal_distribution<float> n_angle_common (0.0f, std_angle_);
      std::normal_distribution<float> n_angle_diff   (0.0f, std_angle_diff_);

      for(int h=0; h<horizon_; h++){

        float tc = n_thrust_common(local_gen);
        float td = n_thrust_diff(local_gen);
        float sc = n_angle_common(local_gen);
        float sd = n_angle_diff(local_gen);

        float thrust_noise_L = tc + td;
        float thrust_noise_R = tc - td;
        float steer_noise_L  = sc + sd;
        float steer_noise_R  = sc - sd;

        // float nominal locals keep the std::clamp types consistent
        float u_nom_0 = U_(0, h);
        float u_nom_1 = U_(1, h);
        float u_nom_2 = U_(2, h);
        float u_nom_3 = U_(3, h);

        float c0 = std::clamp(u_nom_0 + thrust_noise_L, (float)-max_thrust_delta, (float)max_thrust_delta);
        float c1 = std::clamp(u_nom_1 + thrust_noise_R, (float)-max_thrust_delta, (float)max_thrust_delta);
        float c2 = std::clamp(u_nom_2 + steer_noise_L,  (float)-max_angle_delta,  (float)max_angle_delta);
        float c3 = std::clamp(u_nom_3 + steer_noise_R,  (float)-max_angle_delta,  (float)max_angle_delta);

        float n0 = c0 - u_nom_0;
        float n1 = c1 - u_nom_1;
        float n2 = c2 - u_nom_2;
        float n3 = c3 - u_nom_3;

        int idx = k * (horizon_ * 4) + (h * 4);
        h_noise[idx + 0] = n0;
        h_noise[idx + 1] = n1;
        h_noise[idx + 2] = n2;
        h_noise[idx + 3] = n3;

        rollout_noises_[k](0, h) = n0;
        rollout_noises_[k](1, h) = n1;
        rollout_noises_[k](2, h) = n2;
        rollout_noises_[k](3, h) = n3;
      }
    }

    auto t_noise = std::chrono::steady_clock::now();

    // 2. Flatten State, Nominal U, and Path for GPU transfer
    float h_initial_state[10];
    for (int i = 0; i < 6; i++) h_initial_state[i] = current_state_(i);
    for (int i = 0; i < 4; i++) h_initial_state[6 + i] = prev_published_cmd_(i); // Include actuator states!

    std::vector<float> h_nominal_U(horizon_ * 4);
    for (int h = 0; h < horizon_; h++) {
        h_nominal_U[h * 4 + 0] = U_(0, h);
        h_nominal_U[h * 4 + 1] = U_(1, h);
        h_nominal_U[h * 4 + 2] = U_(2, h);
        h_nominal_U[h * 4 + 3] = U_(3, h);
    }

    std::vector<float> h_ref_path(horizon_ * 3);
    for (int h = 0; h < horizon_; h++) {
        h_ref_path[h * 3 + 0] = ref_traj[h](0);
        h_ref_path[h * 3 + 1] = ref_traj[h](1);
        h_ref_path[h * 3 + 2] = ref_traj[h](2);
    }

    std::vector<float> h_costs(K_, 0.0f);

    MPPIParams params;
    params.horizon = horizon_;
    params.dt = dt_;
    params.L = L_;
    params.W = W_;
    params.m_u = m_u_;
    params.m_v = m_v_;
    params.m_r = m_r_;
    params.d_u = d_u_;
    params.d_v = d_v_;
    params.d_r = d_r_;
    params.d_u_quad = d_u_quad_;
    params.d_v_quad = d_v_quad_;
    params.d_r_quad = d_r_quad_;
    params.q_cross = q_cross_;
    params.q_along = q_along_;
    params.q_theta = yaw_weight_;
    params.F_max = 5000.0f;
    params.speed_weight = speed_weight_;
    params.yaw_rate_weight = yaw_rate_weight_;
    params.thrust_weight = thrust_weight_;
    params.steer_weight = steer_weight_;
    params.target_speed = target_speed_;   // <-- fixed cruise target (1.3 m/s)

    // 3. Send to the GPU 
    launch_mppi_cuda(
        h_initial_state, 
        h_nominal_U.data(), 
        h_noise.data(), 
        h_ref_path.data(), 
        h_costs.data(), 
        params, 
        K_
    );

    // 4. Retrieve the costs from GPU and store them in rollout_costs_
    for (int k = 0; k < K_; k++) {
        rollout_costs_[k] = h_costs[k];
    }

    auto t_gpu = std::chrono::steady_clock::now();

    double min_cost_ = *std::min_element(rollout_costs_.begin(), rollout_costs_.end());
    double max_cost_ = *std::max_element(rollout_costs_.begin(), rollout_costs_.end());
    (void)max_cost_;  // currently unused

    // --- 5. VISUALIZE THE ROLL-OUTS IN RVIZ ---
    if (loop_count_++ % 10 == 0) {
    visualization_msgs::msg::MarkerArray rollout_markers;
    
    // FIRST: Send a command to clear the old lines from the previous frame
    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    rollout_markers.markers.push_back(delete_all);

    int num_visualized = 60; // Safe number of tentacles for RViz
    int step = K_ / num_visualized;
    rclcpp::Time current_time = this->now();

    for (int k = 0; k < K_; k += step) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world"; // Ensure this matches your global frame!
        marker.header.stamp = current_time;
        marker.ns = "mppi_rollouts";
        marker.id = k;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05; // Thin lines for the tentacles

        // Make them Cyan and highly transparent so you can see through them
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 0.15f; 

        // Initialize the state for this specific rollout simulation
        Eigen::VectorXd sim_state(10);
        sim_state.head(6) = current_state_;
        sim_state.tail(4) = prev_published_cmd_;

        for (int h = 0; h < horizon_; h++) {
            // Add the current point to the line
            geometry_msgs::msg::Point p;
            p.x = sim_state(0);
            p.y = sim_state(1);
            p.z = 0.0;
            marker.points.push_back(p);
            
            // Reconstruct the exact control the GPU used for this step
            Eigen::VectorXd control(4);
            control(0) = U_(0, h) + rollout_noises_[k](0, h);
            control(1) = U_(1, h) + rollout_noises_[k](1, h);
            control(2) = U_(2, h) + rollout_noises_[k](2, h);
            control(3) = U_(3, h) + rollout_noises_[k](3, h);
            
            // Step the physics forward
            sim_state = predict_state(sim_state, control);
        }
        rollout_markers.markers.push_back(marker);
    }

    // Publish the tentacles to RViz
    pub_current_state_rollouts_->publish(rollout_markers);
    }  // end viz throttle

    auto t_viz = std::chrono::steady_clock::now();

    double sum_weights_ = 0.0;
    std::vector<double> weights_(K_, 0.0);

    for (int k = 0; k < K_; k++) {
        weights_[k] = std::exp(-(rollout_costs_[k] - min_cost_) / lambda_);
        sum_weights_ += weights_[k];
    }

    // Update U_ (which now represents optimal control RATES)
    for (int k = 0; k < K_; k++) {
        weights_[k] /= sum_weights_;
        U_ += update_gain_ * weights_[k] * rollout_noises_[k]; 
    }

    // YOU MISSED THIS TOO! Prevent the internal memory from exploding to infinity over time!
    for (int h = 0; h < horizon_; h++) {
        U_(0, h) = std::clamp(U_(0, h), -max_thrust_delta, max_thrust_delta);
        U_(1, h) = std::clamp(U_(1, h), -max_thrust_delta, max_thrust_delta);
        U_(2, h) = std::clamp(U_(2, h), -max_angle_delta, max_angle_delta);
        U_(3, h) = std::clamp(U_(3, h), -max_angle_delta, max_angle_delta);
    }

    // Extract the optimal rate for the very next time step
    Eigen::VectorXd optimal_rate = U_.col(0);

    // INTEGRATE to get absolute command
    Eigen::VectorXd cmd = prev_published_cmd_ + (optimal_rate * dt_);

    // Clamp the final command to physical limits
    cmd(0) = std::clamp(cmd(0), min_thrusters_clamped, max_thrusters_clamped);
    cmd(1) = std::clamp(cmd(1), min_thrusters_clamped, max_thrusters_clamped);
    cmd(2) = std::clamp(cmd(2), min_rotation_clamped, max_rotation_clamped);
    cmd(3) = std::clamp(cmd(3), min_rotation_clamped, max_rotation_clamped);

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

    // --- Log this cycle's tracking/effort data ---
    // min_dist: perpendicular-ish distance to the nearest path point, computed
    // above in step 1 (closest-point search) -- reused here as cross-track error.
    log_file_ << std::fixed << std::setprecision(4)
              << this->now().seconds() << ","
              << boat_x << "," << boat_y << "," << current_state_(2) << ","
              << current_speed << "," << (target_speed_ - current_speed) << ","
              << min_dist << "," << current_state_(5) << ","
              << msg_fl.data << "," << msg_fr.data << ","
              << msg_al.data << "," << msg_ar.data << ","
              << min_cost_ << "\n";
    log_file_.flush();

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
      "Commanding Thrusters -> L: %.2f N, R: %.2f N | Angles -> L: %.2f rad, R: %.2f rad", msg_fl.data, msg_fr.data, msg_al.data, msg_ar.data);

    nav_msgs::msg::Path optimal_path;
    optimal_path.header.frame_id = "world";
    optimal_path.header.stamp = this->now();
    
    Eigen::VectorXd sim_state(10);
    sim_state.head(6) = current_state_;
    sim_state.tail(4) = prev_published_cmd_;
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

    auto t_end = std::chrono::steady_clock::now();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "TIMING ms | setup:%.1f noise:%.1f gpu:%.1f viz:%.1f rest:%.1f | TOTAL:%.1f (budget %.0f)",
      ms(t_start,t_ref), ms(t_ref,t_noise), ms(t_noise,t_gpu),
      ms(t_gpu,t_viz), ms(t_viz,t_end), ms(t_start,t_end), dt_*1000.0);
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
  size_t loop_count_ = 0;   // for throttling the debug visualization
  
  Eigen::VectorXd current_state_;
  nav_msgs::msg::Path current_plan_;
  Eigen::MatrixXd U_;
  
  std::vector<double> rollout_costs_;
  std::vector<Eigen::MatrixXd> rollout_noises_;
  std::ofstream log_file_;

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
  
  // TUNE PARAMETERS
  // --------------
  const double dt_ = 0.1;
  const int horizon_ = 60;                      
  const int K_ = 4000; 
  const double lambda_ = 40.0;                  
  const double update_gain_ = 0.60;             // <--- LOWERED: Adds natural damping
  const double theta_deadband_ = 0.0;          
  const double min_thrusters_clamped = -0.05;    
  const double max_thrusters_clamped = 0.4;    
  const double min_rotation_clamped = -0.4;     
  const double max_rotation_clamped = 0.4; 
       
  const double max_thrust_delta = 0.09;         // <--- MASSIVE DROP. Physically prevents the engines from jumping to 1700 N instantly.
  const double max_angle_delta = 0.05;          // <--- INCREASED: Let the physical boat turn fast enough!
  const double alpha = 0.6;                     

  // ECONOMY BALANCE (The GPU Setup)
  const double dist_weight_ = 35.0;             // superseded by q_cross_/q_along_ (kept for reference)
  const double q_cross_ = 60.0;                 // <--- CROSS-TRACK: the error that matters (off the line). Weighted hard.
  const double q_along_ = 10.0;                  // <--- ALONG-TRACK: loose, so being slightly ahead/behind doesn't fight the speed critic
  const double yaw_weight_ = 60.0;                          
  
  const double steer_weight_ = 5.0;              // <--- nozzles are now CHEAP so the optimizer vectors instead of differential-thrusting
  const double yaw_rate_weight_ = 300.0;         // <--- Dropped from 300. Let the boat actually rotate when it needs to.

  const double speed_weight_ = 300.0;            // <--- Bumped from 50. Forces it to overcome the doubled drag.
  const double thrust_weight_ = 10.0;           // <--- MASSIVE BUMP. Make engine power expensive! The AI will now be incredibly frugal with the throttle.

  const double std_thrust_ = 0.02;              // <--- DROPPED. With 4,000 rollouts, it doesn't need to imagine huge thrusts to find the sweet spot.
  const double std_angle_ = 0.08;            

  // Differential exploration std-devs (the dial for differential thrust/steering).
  // Keep small: differential is a trim term, common-mode stays primary.
  const double std_thrust_diff_ = 0.02;
  const double std_angle_diff_  = 0.04;

  const double target_speed_ = 1.3;              
  const double ghost_target_speed_ = 1.3; 

  const double var_thrust_ = std_thrust_ * std_thrust_;
  const double var_angle_ = std_angle_ * std_angle_;

  Eigen::VectorXd prev_published_cmd_ = Eigen::VectorXd::Zero(4);
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