#ifndef JETBOT_ODOM_PARAMS_HPP_
#define JETBOT_ODOM_PARAMS_HPP_
/**
 * @file params.hpp
 * @brief Persistent parameter storage (EEPROM) for Teensy wheel odom firmware.
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <stdint.h>

namespace jetbot_odom {

/** @brief Persistent params stored in EEPROM. */
#pragma pack(push, 1) // disable padding so that struct is tightly packed
struct Params {
  uint32_t magic;        ///< Magic number to detect validity.
  uint16_t version;      ///< Params struct version.
  uint16_t reserved;     ///< Padding/reserved.
  float counts_per_rev;  ///< Encoder counts per revolution.
  uint16_t report_hz;    ///< Report rate in Hz.
  int8_t invert_left;    ///< +1 or -1.
  int8_t invert_right;   ///< +1 or -1.
  uint8_t output_mode;   ///< 0=binary, 1=json.
  uint8_t reserved2[7];  ///< Future expansion.
  uint16_t crc16;        ///< CRC16 over bytes [0..offsetof(crc16)-1].
};
#pragma pack(pop)

static constexpr uint32_t kParamsMagic = 0x4A42544FUL; // 'JBT O'
static constexpr uint16_t kParamsVersion = 1;

/**
 * @brief Load parameters from EEPROM into @p p.
 * @param[out] p Filled with loaded params on success.
 * @return true if params were valid and loaded.
 */
bool loadParams(Params& p);

/**
 * @brief Save parameters to EEPROM (updates CRC).
 * @param[in,out] p Params to save (crc16 is written).
 * @return true on success.
 */
bool saveParams(Params& p);

/**
 * @brief Initialize params with safe defaults.
 * @param[out] p Params to initialize.
 */
void defaultParams(Params& p);

} // namespace jetbot_odom

#endif // JETBOT_ODOM_PARAMS_HPP_