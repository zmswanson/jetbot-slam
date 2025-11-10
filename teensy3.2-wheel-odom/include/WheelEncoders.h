#pragma once
/**
 * @file WheelEncoders.h
 * @brief Quadrature encoder interface for left/right wheels.
 *
 * Provides delta tick counts over a configured reporting interval.
 */

#include <Arduino.h>
#include <Encoder.h>

class WheelEncoders {
public:
  /**
   * @brief Construct WheelEncoders.
   * @param leftA Left encoder channel A pin.
   * @param leftB Left encoder channel B pin.
   * @param rightA Right encoder channel A pin.
   * @param rightB Right encoder channel B pin.
   */
  WheelEncoders(uint8_t leftA, uint8_t leftB, uint8_t rightA, uint8_t rightB);

  /** @brief Initialize encoder counters. */
  void begin();

  /**
   * @brief Update internal state; returns true when a new sample is ready.
   *
   * Call this frequently from loop(). When enough time has elapsed, this
   * computes delta ticks since last sample and makes them available through
   * getters.
   *
   * @param samplePeriodUs Sample period in microseconds.
   * @param invertLeft Multiply left delta by +1 or -1.
   * @param invertRight Multiply right delta by +1 or -1.
   * @return true if a new delta sample is ready.
   */
  bool update(uint32_t samplePeriodUs, int8_t invertLeft, int8_t invertRight);

  /** @brief Reset tick accumulators. */
  void reset();

  /** @brief Delta left ticks since last sample. */
  int32_t dlTicks() const { return dl_ticks_; }

  /** @brief Delta right ticks since last sample. */
  int32_t drTicks() const { return dr_ticks_; }

  /** @brief Microseconds since last sample. */
  uint32_t dtUs() const { return dt_us_; }

  /** @brief Total left ticks since begin/reset. */
  int32_t leftTotal() const { return left_total_; }

  /** @brief Total right ticks since begin/reset. */
  int32_t rightTotal() const { return right_total_; }

private:
  Encoder left_;
  Encoder right_;

  int32_t prev_left_{0};
  int32_t prev_right_{0};

  int32_t left_total_{0};
  int32_t right_total_{0};

  uint32_t last_us_{0};

  int32_t dl_ticks_{0};
  int32_t dr_ticks_{0};
  uint32_t dt_us_{0};
};
