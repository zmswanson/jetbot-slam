#ifndef JETBOT_ODOM_CRC16_HPP_
#define JETBOT_ODOM_CRC16_HPP_
/**
 * @file crc16.hpp
 * @brief CRC16-CCITT-FALSE implementation (poly=0x1021, init=0xFFFF).
 */

#include <stdint.h>
#include <stddef.h>

namespace jetbot_odom {

/**
 * @brief Compute CRC16-CCITT-FALSE over a byte buffer.
 * @param data Pointer to bytes.
 * @param len Number of bytes.
 * @return CRC16 value.
 */
static inline uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

} // namespace jetbot_odom

#endif // JETBOT_ODOM_CRC16_HPP_