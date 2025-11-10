#include "WheelEncoders.h"

WheelEncoders::WheelEncoders(uint8_t leftA, uint8_t leftB, uint8_t rightA, uint8_t rightB)
: left_(leftA, leftB), right_(rightA, rightB) {}

void WheelEncoders::begin() {
  prev_left_ = (int32_t)left_.read();
  prev_right_ = (int32_t)right_.read();
  left_total_ = 0;
  right_total_ = 0;
  last_us_ = micros();
  dl_ticks_ = 0;
  dr_ticks_ = 0;
  dt_us_ = 0;
}

void WheelEncoders::reset() {
  begin();
}

bool WheelEncoders::update(uint32_t samplePeriodUs, int8_t invertLeft, int8_t invertRight) {
  const uint32_t now_us = micros();
  const uint32_t elapsed = now_us - last_us_;

  if (elapsed < samplePeriodUs) return false;

  const int32_t l = (int32_t)left_.read();
  const int32_t r = (int32_t)right_.read();

  int32_t dl = l - prev_left_;
  int32_t dr = r - prev_right_;

  prev_left_ = l;
  prev_right_ = r;

  dl *= invertLeft;
  dr *= invertRight;

  dl_ticks_ = dl;
  dr_ticks_ = dr;
  dt_us_ = elapsed;

  left_total_ += dl;
  right_total_ += dr;

  last_us_ = now_us;
  return true;
}
