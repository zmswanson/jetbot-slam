#ifndef JETBOT_ODOM_PROTOCOL_HPP_
#define JETBOT_ODOM_PROTOCOL_HPP_
/**
 * @file protocol.hpp
 * @brief Binary telemetry protocol for Teensy wheel odometry stream.
 *
 * This protocol is designed for robust UART streaming:
 * - Fixed-length frames
 * - Sync word for re-synchronization
 * - Sequence number
 * - CRC16-CCITT for integrity
 *
 * Default stream is binary. A JSON mode is also supported for human debugging.
 */

#include <Arduino.h>
#include <stdint.h>

namespace jetbot_odom {

/** @brief Protocol version. */
static constexpr uint8_t kProtoVersion = 1;

/** @brief Message types. */
enum class MsgType : uint8_t {
  /** @brief Odometry delta ticks message. */
  ODOM_DELTA = 0x01,
};

/** @brief Sync word (little-endian on the wire: 0x55 0xAA). */
static constexpr uint16_t kSyncWord = 0xAA55;

/**
 * @brief Fixed-size odometry delta frame.
 *
 * All fields are little-endian.
 */
#pragma pack(push, 1) // disable padding so that struct is tightly packed
struct OdomDeltaFrame {
  uint16_t sync;      ///< Sync word = kSyncWord.
  uint8_t  version;   ///< Protocol version = kProtoVersion.
  uint8_t  msg_type;  ///< MsgType::ODOM_DELTA (0x01).
  uint32_t seq;       ///< Sequence number (increments each frame).
  uint32_t dt_us;     ///< Microseconds since previous frame.
  int32_t  dl_ticks;  ///< Delta left ticks since previous frame.
  int32_t  dr_ticks;  ///< Delta right ticks since previous frame.
  uint16_t crc16;     ///< CRC16-CCITT over bytes [0..offsetof(crc16)-1].
};
#pragma pack(pop)

/** @brief Size of OdomDeltaFrame in bytes. */
static constexpr size_t kOdomDeltaFrameSize = sizeof(OdomDeltaFrame);

} // namespace jetbot_odom

#endif // JETBOT_ODOM_PROTOCOL_HPP_