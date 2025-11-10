#include "params.hpp"
#include "crc16.hpp"

namespace jetbot_odom {

static constexpr int kEepromAddr = 0;

void defaultParams(Params& p) {
  memset(&p, 0, sizeof(p));
  p.magic = kParamsMagic;
  p.version = kParamsVersion;
  p.counts_per_rev = 1920.0f;
  p.report_hz = 50;
  p.invert_left = 1;
  p.invert_right = 1;
  p.output_mode = 0; // binary
  p.crc16 = 0;
}

static uint16_t compute_crc(const Params& p) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&p);
  const size_t len = offsetof(Params, crc16);
  return crc16_ccitt_false(bytes, len);
}

bool loadParams(Params& p) {
  EEPROM.get(kEepromAddr, p);
  if (p.magic != kParamsMagic) return false;
  if (p.version != kParamsVersion) return false;
  const uint16_t expected = compute_crc(p);
  if (expected != p.crc16) return false;
  // Sanity checks
  if (!(p.invert_left == 1 || p.invert_left == -1)) return false;
  if (!(p.invert_right == 1 || p.invert_right == -1)) return false;
  if (p.report_hz == 0 || p.report_hz > 500) return false;
  if (p.counts_per_rev < 1.0f) return false;
  if (p.output_mode > 1) p.output_mode = 0;
  return true;
}

bool saveParams(Params& p) {
  p.magic = kParamsMagic;
  p.version = kParamsVersion;
  p.crc16 = compute_crc(p);
  EEPROM.put(kEepromAddr, p);
  return true;
}

} // namespace jetbot_odom
