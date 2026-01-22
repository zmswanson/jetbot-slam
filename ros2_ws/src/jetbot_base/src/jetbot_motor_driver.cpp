/**
 * @file jetbot_motor_driver.cpp
 * @brief Implementation of low-level I2C/PWM motor driver for Waveshare JetBot.
 * @ingroup jetbot_base
 */

#include "jetbot_base/jetbot_motor_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <thread>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace jetbot_base {

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

// -------------------------
// I2CDevice
// -------------------------

I2CDevice::I2CDevice(const std::string& dev, uint8_t addr) : addr_(addr) {
  fd_ = ::open(dev.c_str(), O_RDWR);
  if (fd_ < 0) {
    throw std::runtime_error(
      "I2CDevice: failed to open " + dev + ": " + std::string(std::strerror(errno)));
  }

  if (ioctl(fd_, I2C_SLAVE, addr_) < 0) {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error(
      "I2CDevice: failed to set slave address: " + std::string(std::strerror(errno)));
  }
}

I2CDevice::~I2CDevice() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void I2CDevice::writeReg(uint8_t reg, uint8_t value) {
  uint8_t buf[2] = {reg, value};
  if (::write(fd_, buf, 2) != 2) {
    throw std::runtime_error(
      "I2CDevice::writeReg failed: " + std::string(std::strerror(errno)));
  }
}

void I2CDevice::writeRegs(uint8_t reg, const uint8_t* data, std::size_t n) {
  // Keep buffer small and stack-allocated; PCA9685 writes are 4 bytes typically.
  if (n > 16) {
    throw std::runtime_error("I2CDevice::writeRegs: n too large");
  }

  uint8_t buf[1 + 16];
  buf[0] = reg;
  std::memcpy(buf + 1, data, n);

  const ssize_t expected = static_cast<ssize_t>(1 + n);
  if (::write(fd_, buf, 1 + n) != expected) {
    throw std::runtime_error(
      "I2CDevice::writeRegs failed: " + std::string(std::strerror(errno)));
  }
}

// -------------------------
// PCA9685
// -------------------------

PCA9685::PCA9685(const std::string& i2c_dev, uint8_t addr)
  : i2c_(i2c_dev, addr) {}

void PCA9685::init(uint16_t pwm_hz) {
  constexpr uint8_t MODE1    = 0x00;
  constexpr uint8_t PRESCALE = 0xFE;

  // Put device to sleep while updating prescaler (per datasheet).
  i2c_.writeReg(MODE1, 0x10);  // SLEEP=1

  // prescale = round(25MHz / (4096 * freq)) - 1
  double prescale = std::round(25000000.0 / (4096.0 * pwm_hz) - 1.0);
  prescale = clampd(prescale, 3.0, 255.0);
  i2c_.writeReg(PRESCALE, static_cast<uint8_t>(prescale));

  // Wake device and enable auto-increment (AI=1).
  i2c_.writeReg(MODE1, 0x20);
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
}

void PCA9685::setPWM(uint8_t channel, uint16_t on, uint16_t off) {
  // LED0_ON_L starts at 0x06, each channel uses 4 registers.
  const uint8_t base = static_cast<uint8_t>(0x06 + 4 * channel);

  // The PCA9685 uses 12-bit values; upper nibble is in *_H registers.
  const uint8_t data[4] = {
    static_cast<uint8_t>(on & 0xFF),
    static_cast<uint8_t>((on >> 8) & 0x0F),
    static_cast<uint8_t>(off & 0xFF),
    static_cast<uint8_t>((off >> 8) & 0x0F),
  };

  i2c_.writeRegs(base, data, 4);
}

// -------------------------
// MotorHATMotors
// -------------------------

MotorHATMotors::MotorHATMotors(const std::string& i2c_dev, uint8_t addr)
  : pca_(i2c_dev, addr)
{
  // Default Adafruit MotorHAT mapping
  m1_ = MotorMap{8, 10, 9};
  m2_ = MotorMap{13, 11, 12};
}

void MotorHATMotors::init(uint16_t pwm_hz) {
  pca_.init(pwm_hz);
  stopAll();
}

void MotorHATMotors::setMotorChannels(int motor_index,
                                      uint8_t pwm_channel,
                                      uint8_t in1_channel,
                                      uint8_t in2_channel)
{
  if (motor_index == 1) {
    m1_ = MotorMap{pwm_channel, in1_channel, in2_channel};
  } else if (motor_index == 2) {
    m2_ = MotorMap{pwm_channel, in1_channel, in2_channel};
  } else {
    throw std::runtime_error("MotorHATMotors::setMotorChannels: motor_index must be 1 or 2");
  }
}

void MotorHATMotors::setPinHigh(uint8_t ch) {
  // Full on: set ON=4096 bit via registers is one method, but simplest is ON=0, OFF=4095.
  pca_.setPWM(ch, 0, 4095);
}

void MotorHATMotors::setPinLow(uint8_t ch) {
  pca_.setPWM(ch, 0, 0);
}

void MotorHATMotors::setMotor(int motor_index, double cmd) {
  cmd = clampd(cmd, -1.0, 1.0);

  MotorMap m;
  if (motor_index == 1) m = m1_;
  else if (motor_index == 2) m = m2_;
  else throw std::runtime_error("MotorHATMotors::setMotor: motor_index must be 1 or 2");

  // Speed PWM on enable channel
  const double duty = std::abs(cmd);
  int off = static_cast<int>(std::round(duty * 4095.0));
  off = std::max(0, std::min(off, 4095));
  pca_.setPWM(m.pwm, 0, static_cast<uint16_t>(off));

  // Direction pins
  if (cmd > 0) {
    // Forward: IN1 high, IN2 low
    setPinHigh(m.in1);
    setPinLow(m.in2);
  } else if (cmd < 0) {
    // Backward: IN1 low, IN2 high
    setPinLow(m.in1);
    setPinHigh(m.in2);
  } else {
    // Brake/coast behavior: set both low (coast)
    setPinLow(m.in1);
    setPinLow(m.in2);
    pca_.setPWM(m.pwm, 0, 0);
  }
}

void MotorHATMotors::stopAll() {
  // Stop motor 1
  pca_.setPWM(m1_.pwm, 0, 0);
  setPinLow(m1_.in1);
  setPinLow(m1_.in2);

  // Stop motor 2
  pca_.setPWM(m2_.pwm, 0, 0);
  setPinLow(m2_.in1);
  setPinLow(m2_.in2);
}


}  // namespace jetbot_base
