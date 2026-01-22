/**
 * @file jetbot_motor_driver_node.cpp
 * @brief ROS 2 node that converts cmd_vel into Waveshare JetBot motor commands.
 * @ingroup jetbot_base
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "jetbot_base/jetbot_motor_driver.hpp"

using namespace std::chrono_literals;

/**
 * @brief Helper function to clamp a double value between lo and hi.
 *
 * @param x  The value to clamp.
 * @param lo The lower bound.
 * @param hi The upper bound.
 * @return   The clamped value.
 */
static inline double clampd(double x, double lo, double hi) {
  return std::max(lo, std::min(hi, x));
}

/**
 * @brief ROS 2 motor driver node for the Waveshare JetBot.
 *
 * Subscribes to geometry_msgs::msg::Twist on `cmd_vel` and converts it to
 * left/right motor commands for a differential-drive base.
 *
 * Safety / robustness features:
 *  - Command watchdog timeout: stops motors if cmd_vel is stale.
 *  - Acceleration limiting in linear and angular velocities.
 *  - Motors are stopped on node shutdown.
 *
 * Coordinate conventions:
 *  - +linear.x  -> forward
 *  - +angular.z -> counter-clockwise rotation
 *
 * @ingroup jetbot_base
 */
class JetbotMotorDriverNode : public rclcpp::Node {
public:
  /**
   * @brief Construct the motor driver node.
   *
   * Declares ROS parameters, initializes the I2C motor driver, and starts a
   * periodic control loop timer.
   */
  JetbotMotorDriverNode()
  : Node("jetbot_motor_driver")
  {
    // I2C configuration
    // i2c_device: Linux I2C device path (e.g. /dev/i2c-1)
    // i2c_addr:   7-bit I2C address of PCA9685 (default 0x60)
    i2c_device_ = declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
    i2c_addr_   = static_cast<uint8_t>(declare_parameter<int>("i2c_addr", 0x60));

    // Differential drive parameters
    wheel_sep_  = declare_parameter<double>("wheel_separation", 0.10);

    // Command limits (used both for clamping and normalization)
    max_v_      = declare_parameter<double>("max_linear_mps", 0.4);
    max_w_      = declare_parameter<double>("max_angular_rps", 3.0);

    // Safety watchdog timeout (seconds)
    timeout_    = declare_parameter<double>("cmd_timeout_sec", 0.5);

    // Motor inversion flags (useful for wiring/convention differences)
    left_inv_   = declare_parameter<bool>("left_inverted", false);
    right_inv_  = declare_parameter<bool>("right_inverted", false);

    // Slew limits on v and w
    max_accel_  = declare_parameter<double>("max_accel_mps2", 1.0);
    max_alpha_  = declare_parameter<double>("max_alpha_rps2", 6.0);

    // Initialize low-level motor driver
    motors_ = std::make_unique<jetbot_base::MotorHATMotors>(i2c_device_, i2c_addr_);
    motors_->init(1600);

    // Subscribe to cmd_vel. We latch target values and apply them in a timer.
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_time_ = now();
        target_v_ = msg->linear.x;
        target_w_ = msg->angular.z;

        RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "cmd_vel received: v=%.3f w=%.3f",
          target_v_,
          target_w_);
      });

    // Control loop timer (50 Hz)
    last_cmd_time_ = now();
    last_update_time_ = now();
    timer_ = create_wall_timer(20ms, std::bind(&JetbotMotorDriverNode::update, this));

    RCLCPP_INFO(get_logger(),
      "Started. Subscribing to /cmd_vel. I2C device=%s addr=0x%02X",
      i2c_device_.c_str(), i2c_addr_);
  }

  /**
   * @brief Destructor ensures motors are stopped.
   */
  ~JetbotMotorDriverNode() override {
    try {
      if (motors_) {
        motors_->stopAll();
      }
    } catch (...) {
      // Best-effort shutdown; avoid throwing from destructor.
    }
  }

private:
  /**
   * @brief Periodic control loop that applies the latest cmd_vel.
   *
   * Implements:
   *  - watchdog timeout
   *  - clamping of target v/w
   *  - slew-rate limiting (accel/alpha)
   *  - differential drive mixing into left/right commands
   *  - normalization into [-1, 1] motor commands
   */
  void update() {
    const auto t = now();
    const double dt = (t - last_update_time_).seconds();
    last_update_time_ = t;

    // If cmd_vel is stale, stop motors and reset internal velocity state.
    if ((t - last_cmd_time_).seconds() > timeout_) {
      motors_->stopAll();
      cur_v_ = 0.0;
      cur_w_ = 0.0;
      return;
    }

    // Clamp requested commands
    const double v_req = clampd(target_v_, -max_v_, max_v_);
    const double w_req = clampd(target_w_, -max_w_, max_w_);

    // Apply slew-rate limiting (simple first-order limiter)
    if (dt > 0.0) {
      cur_v_ += clampd(v_req - cur_v_, -max_accel_ * dt,  max_accel_ * dt);
      cur_w_ += clampd(w_req - cur_w_, -max_alpha_ * dt,  max_alpha_ * dt);
    } else {
      cur_v_ = v_req;
      cur_w_ = w_req;
    }

    // Differential drive mixing
    const double v_l = cur_v_ - cur_w_ * (wheel_sep_ / 2.0);
    const double v_r = cur_v_ + cur_w_ * (wheel_sep_ / 2.0);

    // Normalize to motor command space using max_v_ as scale
    double cmd_l = (max_v_ > 1e-6) ? (v_l / max_v_) : 0.0;
    double cmd_r = (max_v_ > 1e-6) ? (v_r / max_v_) : 0.0;

    cmd_l = clampd(cmd_l, -1.0, 1.0);
    cmd_r = clampd(cmd_r, -1.0, 1.0);

    // Apply optional inversion
    if (left_inv_)  { cmd_l = -cmd_l; }
    if (right_inv_) { cmd_r = -cmd_r; }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "motor cmd: L=%.2f R=%.2f (cur_v=%.2f cur_w=%.2f)",
      cmd_l,
      cmd_r,
      cur_v_,
      cur_w_
    );

    // Waveshare mapping: left motor is channel 1, right motor is channel 2
    motors_->setMotor(1, cmd_l);
    motors_->setMotor(2, cmd_r);
  }

  // Low-level driver
  std::unique_ptr<jetbot_base::MotorHATMotors> motors_;

  // Parameters
  std::string i2c_device_;
  uint8_t i2c_addr_{0x60};

  double wheel_sep_{0.10};
  double max_v_{0.4};
  double max_w_{3.0};
  double timeout_{0.5};
  bool left_inv_{false};
  bool right_inv_{false};
  double max_accel_{1.0};
  double max_alpha_{6.0};

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Timing/state
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_update_time_;
  double target_v_{0.0};
  double target_w_{0.0};
  double cur_v_{0.0};
  double cur_w_{0.0};
};

/**
 * @brief Main entry point for the motor driver node.
 *
 * Initializes ROS 2, spins the node, and performs a clean shutdown.
 *
 * @param argc Standard argc.
 * @param argv Standard argv.
 * @return Process return code.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JetbotMotorDriverNode>());
  rclcpp::shutdown();
  return 0;
}
