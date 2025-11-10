#pragma once
/**
 * @file wheel_odom_node.hpp
 * @brief ROS 2 node that consumes Teensy wheel encoder delta ticks over serial and publishes wheel odometry.
 *
 * ## Overview
 * The Teensy is expected to stream fixed-length binary frames that include:
 * - Sequence number
 * - dt (microseconds since last frame)
 * - delta left ticks
 * - delta right ticks
 * - CRC16 for integrity
 *
 * This node:
 * - Reads and parses frames from a serial port (default: /dev/ttyTHS1)
 * - Integrates pose from wheel delta distances using midpoint integration
 * - Publishes raw wheel odometry as nav_msgs/msg/Odometry on /wheel/odom (configurable)
 * - Optionally broadcasts TF odom->base_link (default: false; recommended false when using robot_localization)
 * - Publishes diagnostic_msgs/DiagnosticArray with packet/CRC/age stats
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace jetbot_base {

/** @brief Output mode of Teensy (kept for future; ROS always parses binary). */
enum class TeensyMode : uint8_t { BIN = 0, JSON = 1 };

/**
 * @brief ROS 2 node implementing wheel odometry from Teensy delta tick frames.
 */
class WheelOdomNode final : public rclcpp::Node {
public:
  explicit WheelOdomNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~WheelOdomNode() override;

  WheelOdomNode(const WheelOdomNode&) = delete;
  WheelOdomNode& operator=(const WheelOdomNode&) = delete;

private:
  // === Serial reader thread ===
  void readerLoop();
  bool openSerial();
  void closeSerial();
  void resyncBuffer();
  void handleFrame(uint32_t seq, uint32_t dt_us, int32_t dl_ticks, int32_t dr_ticks);

  // === Publishing ===
  void publishOdom(const rclcpp::Time& stamp, double v, double w);
  void publishDiagnostics();

  // === CRC ===
  static uint16_t crc16_ccitt_false(const uint8_t* data, size_t len);

  // === Parameters ===
  std::string port_;
  int baud_{115200};

  // Wheel geometry + encoder scale
  double wheel_radius_m_{0.0325};
  double wheel_base_m_{0.120};
  double ticks_per_rev_{1920.0};

  // Optional per-wheel sign (keep for safety)
  int left_sign_{1};
  int right_sign_{1};

  // Frames / topics
  std::string frame_id_{"odom"};
  std::string child_frame_id_{"base_link"};
  std::string odom_topic_{"/wheel/odom"};
  bool publish_tf_{false};

  // Covariance (diagonal)
  std::vector<double> pose_cov_diag_;
  std::vector<double> twist_cov_diag_;

  // === State ===
  std::mutex state_mtx_;
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  // last sequence tracking
  std::atomic<uint32_t> last_seq_{0};
  std::atomic<bool> have_seq_{false};

  // timing stats
  std::atomic<int64_t> last_rx_ns_{0};
  std::atomic<double> last_v_{0.0};
  std::atomic<double> last_w_{0.0};
  std::atomic<int32_t> last_dl_{0};
  std::atomic<int32_t> last_dr_{0};
  std::atomic<uint32_t> last_dt_us_{0};

  // counters
  std::atomic<uint64_t> bytes_read_{0};
  std::atomic<uint64_t> frames_ok_{0};
  std::atomic<uint64_t> crc_fail_{0};
  std::atomic<uint64_t> resync_count_{0};
  std::atomic<uint64_t> seq_jumps_{0};

  // reader thread management
  int fd_{-1};
  std::atomic<bool> stop_{false};
  std::thread reader_;
  std::vector<uint8_t> rx_buf_;

  // ROS interfaces
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

}  // namespace jetbot_base
