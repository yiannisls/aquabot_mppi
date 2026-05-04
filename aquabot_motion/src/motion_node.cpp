#include <chrono>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Dense>


using std::placeholders::_1;
using namespace std::chrono_literals;

class AquabotController : public rclcpp::Node {
public:
  AquabotController() : Node("aquabot_controller") {

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/aquabot/odom", 10, std::bind(&AquabotController::odom_callback, this, std::placeholders::_1));

    // rviz 2D goal pose
    // plan_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    //   "/plan", 10, std::bind(&AquabotController::plan_callback, this, std::placeholders::_1));
    goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&AquabotController::goal_callback, this, _1));

    pub_fl_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/left/thrust", 10);
    pub_fr_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/right/thrust", 10);
    pub_al_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/left/cmd_pos", 10);
    pub_ar_ = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/right/cmd_pos", 10);

    //10 Hz timer
    timer_ = this->create_wall_timer(100ms, std::bind(&AquabotController::control_loop, this));
    
    // flags to not start computing
    has_odom_=false;
    gole_recieved_=false;
  }

private:

  Eigen::VectorXd predict_state(const Eigen::VectorXd& state, const Eigen::VectorXd& control) {

    double x = state(0);
    double y = state(1);
    double yaw = state(2);
    double u = state(3);
    double v = state(4);
    double r = state(5);

    
    double F_L = control(0);
    double F_R = control(1);
    double alpha_L = control(2);
    double alpha_R = control(3);

    // tau = [X, Y, N]^T

    double tau_X = (F_L * std::cos(alpha_L)) + (F_R * std::cos(alpha_R));
    double tau_Y = (F_L * std::sin(alpha_L)) + (F_R * std::sin(alpha_R));
    double tau_N = L_/(2*(F_R * std::cos(alpha_L)) - (F_L * std::cos(alpha_R)) - W_*((F_L * std::sin(alpha_L)) + (F_R * std::sin(alpha_R))));// ... your turn using L_ and W_ ...

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

    // T
    // nrmalize the new yaw angle to stay between -PI and PI
    // This is what the professor said to stay betwen the pi's
    next_state(2) = std::atan2(std::sin(next_state(2)), std::cos(next_state(2)));

    return next_state;
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Update our target variables with the click from RViz2
    goal_x = msg->pose.position.x;
    goal_y = msg->pose.position.y;
    
    goal_recieved_ = true;

    // print on terminal the pose
    RCLCPP_INFO(this->get_logger(), "New Goal Received: X: %.2f, Y: %.2f", goal_x, goal_y);
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
  // TODO() Implement the cost_function

  void cost_function(){

  }
  }
  void control_loop(const nav_msgs::msg::Odometry::SharedPtr msg) {
  
  

    
    if(!goal_recieved_ || !has_odom_){ 
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for Odometry and Goal...");

      return; 
    }       

    clcpp::TimerBase::SharedPtr timer_;
    // TODO() Right
    // publishing a value to show it is working
    auto cmd = std_msgs::msg::Float64();

    cmd.data = 50.0; // Increased power so you can see movement
    pub_fl_->publish(cmd);
    pub_fr_->publish(cmd);
    pub_rl_->publish(cmd);
    pub_rr_->publish(cmd);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fl_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fr_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_al_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ar_;

  // MPPI parameters
  auto dt_ = 0.1; // 10 Hz
  auto horizon_ = 40; // 0.1X40 = 4 sec prediction horizon
  auto K_ = 500; //rollouts
  
  // TODO tune lambda later for softmax
  auto lambda_ = 1.0 // softmax weighting
  // Physical parameters (from URDF)
  const double L_ = 3.0;
  const double W_ = 0.6;
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
