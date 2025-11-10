#include "jetbot_base/wheel_odom_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace jetbot_base {

namespace {

// Wire format matches Teensy OdomDeltaFrame (22 bytes).
static constexpr uint16_t kSyncWord = 0xAA55;
static constexpr uint8_t  kProtoVersion = 1;
static constexpr uint8_t  kMsgTypeOdomDelta = 0x01;
static constexpr size_t   kFrameSize = 22;

/**
 * @brief Odometry delta frame structure (packed).
 * @note Must match Teensy definition!
 */
#pragma pack(push, 1)
struct OdomDeltaFrame {
  uint16_t sync;
  uint8_t  version;
  uint8_t  msg_type;
  uint32_t seq;
  uint32_t dt_us;
  int32_t  dl_ticks;
  int32_t  dr_ticks;
  uint16_t crc16;
};
#pragma pack(pop)

static_assert(sizeof(OdomDeltaFrame) == kFrameSize, "OdomDeltaFrame size mismatch");

/**
 * @brief Convert integer baud rate to termios speed_t.
 * @throws std::runtime_error if unsupported baud rate.
 */
speed_t toSpeed(int baud) {
  switch (baud) {
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 921600: return B921600;
    default: throw std::runtime_error("Unsupported baud rate");
  }
}

/**
 * @brief Get steady clock time in nanoseconds.
 */
int64_t nowSteadyNs() {
  using namespace std::chrono;
  return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

}  // namespace

/**
 * @brief Compute CRC-16-CCITT-FALSE over data.
 * @param data Pointer to data.
 * @param len Length of data in bytes.
 * @return Computed CRC16 value.
 * 
 * @note Polynomial 0x1021, initial value 0xFFFF, no reflection, no final XOR.
 * @note Must match Teensy implementation!
 * @see https://en.wikipedia.org/wiki/Computation_of_cyclic_redundancy_checks
 */
uint16_t WheelOdomNode::crc16_ccitt_false(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x8000) crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      else crc = static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

WheelOdomNode::WheelOdomNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("jetbot_wheel_odom", options) {

  // === Serial Parameters ===
  port_ = declare_parameter<std::string>("port", "/dev/ttyTHS1");
  baud_ = declare_parameter<int>("baud", 115200);

  // === Wheel Parameters ===
  wheel_radius_m_ = declare_parameter<double>("wheel_radius_m", 0.0325);
  wheel_base_m_   = declare_parameter<double>("wheel_base_m", 0.120);
  ticks_per_rev_  = declare_parameter<double>("ticks_per_rev", 1920.0);

  left_sign_  = declare_parameter<int>("left_sign", 1);
  right_sign_ = declare_parameter<int>("right_sign", 1);

  // === Frame / Topic Parameters ===
  frame_id_       = declare_parameter<std::string>("frame_id", "odom");
  child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
  odom_topic_     = declare_parameter<std::string>("odom_topic", "/wheel/odom");
  publish_tf_     = declare_parameter<bool>("publish_tf", false);

  pose_cov_diag_  = declare_parameter<std::vector<double>>("pose_cov_diag",
                    std::vector<double>{1e-3, 1e-3, 1e6, 1e6, 1e6, 1e-2});
  twist_cov_diag_ = declare_parameter<std::vector<double>>("twist_cov_diag",
                    std::vector<double>{1e-2, 1e6, 1e6, 1e6, 1e6, 1e-1});

  // Validate parameters now
  if (pose_cov_diag_.size() != 6 || twist_cov_diag_.size() != 6) {
    throw std::runtime_error("pose_cov_diag and twist_cov_diag must be length 6");
  }
  if (!(left_sign_ == 1 || left_sign_ == -1) || !(right_sign_ == 1 || right_sign_ == -1)) {
    throw std::runtime_error("left_sign/right_sign must be +1 or -1");
  }
  if (ticks_per_rev_ <= 0.0) {
    throw std::runtime_error("ticks_per_rev must be > 0");
  }


  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(20));
  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Diagnostics at 1 Hz
  diag_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&WheelOdomNode::publishDiagnostics, this));

  // Reserve RX buffer--given frame size and typical rates, 4k should be plenty.
  // Check here if start running into memory issues or missing data.
  rx_buf_.reserve(4096);

  // Start serial reader in a background thread--ensure that destructor correctly joins it.
  if (!openSerial()) {
    RCLCPP_WARN(get_logger(), "Failed to open serial %s, will retry in background", port_.c_str());
  }
  reader_ = std::thread(&WheelOdomNode::readerLoop, this);

  RCLCPP_INFO(get_logger(),
    "WheelOdom: port=%s baud=%d ticks_per_rev=%.2f R=%.4f B=%.4f publish_tf=%s topic=%s",
    port_.c_str(), baud_, ticks_per_rev_, wheel_radius_m_, wheel_base_m_, publish_tf_ ? "true" : "false",
    odom_topic_.c_str());
}


WheelOdomNode::~WheelOdomNode() {
  stop_.store(true);
  if (reader_.joinable()) reader_.join();
  closeSerial();
}


bool WheelOdomNode::openSerial() {
  // Close existing if any open to ensure clean state
  closeSerial();

  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) return false;

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) { closeSerial(); return false; }
  cfmakeraw(&tty);

  const speed_t spd = toSpeed(baud_);
  cfsetispeed(&tty, spd);
  cfsetospeed(&tty, spd);

  tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
  tty.c_cflag &= ~CRTSCTS;         // no hardware flow control
  tty.c_cflag &= ~CSTOPB;          // 1 stop bit
  tty.c_cflag &= ~PARENB;          // no parity
  tty.c_cflag &= ~CSIZE;           // clear data bits setting
  tty.c_cflag |= CS8;              // 8 data bits

  tty.c_cc[VMIN]  = 0; // non-blocking read
  tty.c_cc[VTIME] = 1; // 100ms

  // Apply settings; immediately close on failure
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) { 
    closeSerial(); 
    return false;
  }

  return true;
}

void WheelOdomNode::closeSerial() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void WheelOdomNode::resyncBuffer() {
  // Keep last 1 byte to help find sync across reads.
  if (rx_buf_.size() > 1) {
    rx_buf_.erase(rx_buf_.begin(), rx_buf_.end() - 1);
  }
  resync_count_.fetch_add(1);
}

void WheelOdomNode::readerLoop() {
  using namespace std::chrono_literals;

  std::vector<uint8_t> tmp(512);

  while (!stop_.load()) {
    if (fd_ < 0) {
      // Retry open
      if (!openSerial()) {
        std::this_thread::sleep_for(500ms);
        continue;
      }
      // New port opened, clear buffer
      rx_buf_.clear();
    }

    const int n = ::read(fd_, tmp.data(), (int)tmp.size());
    if (n < 0) {
      // Error: close and retry
      closeSerial();
      std::this_thread::sleep_for(200ms);
      continue;
    }
    if (n == 0) {
      // timeout, loop
      continue;
    }

    bytes_read_.fetch_add((uint64_t)n);
    last_rx_ns_.store(nowSteadyNs());

    rx_buf_.insert(rx_buf_.end(), tmp.begin(), tmp.begin() + n);

    // Parse frames
    while (rx_buf_.size() >= kFrameSize) {
      // Find sync
      size_t i = 0;
      bool found = false;
      for (; i + 1 < rx_buf_.size(); ++i) {
        const uint16_t s = (uint16_t)rx_buf_[i] | ((uint16_t)rx_buf_[i + 1] << 8);
        if (s == kSyncWord) { found = true; break; }
      }
      if (!found) {
        resyncBuffer();
        break;
      }
      if (i > 0) {
        rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + (long)i);
        continue;
      }
      if (rx_buf_.size() < kFrameSize) break;

      OdomDeltaFrame f{};
      std::memcpy(&f, rx_buf_.data(), kFrameSize);

      // Basic header checks
      if (f.sync != kSyncWord || f.version != kProtoVersion || f.msg_type != kMsgTypeOdomDelta) {
        rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + 1);
        resync_count_.fetch_add(1);
        continue;
      }

      const uint16_t expected = crc16_ccitt_false(reinterpret_cast<const uint8_t*>(&f), offsetof(OdomDeltaFrame, crc16));
      if (expected != f.crc16) {
        crc_fail_.fetch_add(1);
        // Drop one byte and rescan
        rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + 1);
        continue;
      }

      // Consume frame
      rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + (long)kFrameSize);
      frames_ok_.fetch_add(1);

      handleFrame(f.seq, f.dt_us, f.dl_ticks, f.dr_ticks);
    }
  }
}

void WheelOdomNode::handleFrame(uint32_t seq, uint32_t dt_us, int32_t dl_ticks, int32_t dr_ticks) {
  // Sequence tracking
  if (have_seq_.load()) {
    const uint32_t prev = last_seq_.load();
    if (seq != prev + 1) seq_jumps_.fetch_add(1);
  } else {
    have_seq_.store(true);
  }
  last_seq_.store(seq);

  // Apply sign (safety)
  dl_ticks *= left_sign_;
  dr_ticks *= right_sign_;

  last_dl_.store(dl_ticks);
  last_dr_.store(dr_ticks);
  last_dt_us_.store(dt_us);

  if (dt_us == 0) return;

  const double dt = static_cast<double>(dt_us) * 1e-6;
  if (dt <= 0.0 || dt > 1.0) {
    // Drop insane dt (e.g., unplug/replug)
    return;
  }

  // Convert ticks -> distance per wheel
  const double meters_per_tick = (2.0 * M_PI * wheel_radius_m_) / ticks_per_rev_;
  const double dL = static_cast<double>(dl_ticks) * meters_per_tick;
  const double dR = static_cast<double>(dr_ticks) * meters_per_tick;

  const double dS = 0.5 * (dR + dL);
  const double dTheta = (dR - dL) / wheel_base_m_;

  // Midpoint integration (better than Euler)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    const double yaw_mid = yaw_ + 0.5 * dTheta;
    x_   += dS * std::cos(yaw_mid);
    y_   += dS * std::sin(yaw_mid);
    yaw_ += dTheta;
  }

  // Compute linear (v) and angular (w) velocity
  const double v = dS / dt;
  const double w = dTheta / dt;

  last_v_.store(v);
  last_w_.store(w);

  publishOdom(this->now(), v, w);
}

void WheelOdomNode::publishOdom(const rclcpp::Time& stamp, double v, double w) {
  nav_msgs::msg::Odometry odom{};
  odom.header.stamp = stamp;
  odom.header.frame_id = frame_id_;
  odom.child_frame_id = child_frame_id_;

  double x, y, yaw;
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    x = x_; y = y_; yaw = yaw_;
  }

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;

  // yaw -> quaternion
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = std::sin(yaw * 0.5);
  odom.pose.pose.orientation.w = std::cos(yaw * 0.5);

  odom.twist.twist.linear.x = v;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = w;

  // Fill covariances (diag only)
  for (int i = 0; i < 36; ++i) {
    odom.pose.covariance[i] = 0.0;
    odom.twist.covariance[i] = 0.0;
  }
  for (int i = 0; i < 6; ++i) {
    odom.pose.covariance[i * 6 + i] = pose_cov_diag_[i];
    odom.twist.covariance[i * 6 + i] = twist_cov_diag_[i];
  }

  odom_pub_->publish(odom);

  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf{};
    tf.header.stamp = stamp;
    tf.header.frame_id = frame_id_;
    tf.child_frame_id = child_frame_id_;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }
}

void WheelOdomNode::publishDiagnostics() {
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "jetbot_wheel_odom";
  st.hardware_id = port_;

  // Basic health: if no packets in > 2s -> WARN
  const int64_t last_ns = last_rx_ns_.load();
  const int64_t now_ns = nowSteadyNs();
  const double age_s = (last_ns > 0) ? (double)(now_ns - last_ns) * 1e-9 : 1e9;

  if (age_s < 2.0) st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  else if (age_s < 5.0) st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  else st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;

  st.message = (age_s < 2.0) ? "Receiving frames" : "Stale or no frames";

  auto kv = [&](const std::string& k, const std::string& v) {
    diagnostic_msgs::msg::KeyValue x;
    x.key = k;
    x.value = v;
    st.values.push_back(x);
  };

  kv("bytes_read", std::to_string(bytes_read_.load()));
  kv("frames_ok", std::to_string(frames_ok_.load()));
  kv("crc_fail", std::to_string(crc_fail_.load()));
  kv("resync_count", std::to_string(resync_count_.load()));
  kv("seq_jumps", std::to_string(seq_jumps_.load()));
  kv("last_packet_age_s", std::to_string(age_s));
  kv("last_dl", std::to_string(last_dl_.load()));
  kv("last_dr", std::to_string(last_dr_.load()));
  kv("last_dt_us", std::to_string(last_dt_us_.load()));
  kv("last_v_mps", std::to_string(last_v_.load()));
  kv("last_w_rps", std::to_string(last_w_.load()));

  arr.status.push_back(st);
  diag_pub_->publish(arr);
}

}  // namespace jetbot_base
