/**
 * @file jetbot_motor_driver.hpp
 * @brief Low-level I2C/PWM motor driver for the Waveshare JetBot motor board.
 *
 * This file provides a minimal Linux I2C wrapper, a PCA9685 PWM driver, and a
 * Waveshare JetBot motor mapping that matches the behavior of the original
 * JetBot Python implementation (MotorHAT-style with INA/INB PWM control).
 *
 * @ingroup jetbot_base
 */

#ifndef JETBOT_MOTOR_DRIVER_HPP__
#define JETBOT_MOTOR_DRIVER_HPP__

#include <cstddef>
#include <cstdint>
#include <string>

namespace jetbot_base {

/**
 * @defgroup jetbot_base JetBot Base Drivers
 * @brief Base-level drivers and utilities for the JetBot platform.
 *
 * This group contains low-level components that interface directly with JetBot
 * hardware (motors, encoders, etc.) and the ROS 2 nodes that expose that
 * functionality.
 */

/**
 * @brief Minimal Linux I2C device wrapper.
 *
 * Encapsulates opening a Linux I2C device node (e.g. /dev/i2c-1), setting the
 * active 7-bit slave address, and writing registers.
 *
 * This class owns the file descriptor (RAII) and is non-copyable.
 *
 * @ingroup jetbot_base
 */
class I2CDevice {
public:
  /**
   * @brief Construct an I2C device wrapper for a given bus and slave address.
   *
   * Opens the device node and configures the active I2C slave address for
   * subsequent transactions on this file descriptor.
   *
   * @param dev  Path to the I2C device node (e.g. "/dev/i2c-1").
   * @param addr 7-bit I2C slave address.
   *
   * @throws std::runtime_error on failure to open the device or set the address.
   */
  I2CDevice(const std::string& dev, uint8_t addr);

  /**
   * @brief Destructor closes the I2C file descriptor.
   */
  ~I2CDevice();

  I2CDevice(const I2CDevice&) = delete;
  I2CDevice& operator=(const I2CDevice&) = delete;

  /**
   * @brief Write a single byte to a device register.
   *
   * @param reg   Register address.
   * @param value Value to write.
   *
   * @throws std::runtime_error on I2C write failure.
   */
  void writeReg(uint8_t reg, uint8_t value);

  /**
   * @brief Write multiple bytes starting at a device register.
   *
   * This issues a combined write of the register address followed by @p n bytes
   * of payload. Many I2C peripherals (including the PCA9685) use this pattern
   * for register writes.
   *
   * @param reg  Start register address.
   * @param data Pointer to data bytes.
   * @param n    Number of bytes to write.
   *
   * @throws std::runtime_error on I2C write failure.
   */
  void writeRegs(uint8_t reg, const uint8_t* data, std::size_t n);

private:
  int fd_{-1};
  uint8_t addr_{0};
};

/**
 * @brief Minimal PCA9685 16-channel, 12-bit PWM driver.
 *
 * Provides initialization and per-channel PWM writes suitable for controlling
 * the Waveshare JetBot motor board.
 *
 * @ingroup jetbot_base
 */
class PCA9685 {
public:
  /**
   * @brief Construct a PCA9685 driver on a given I2C bus and address.
   *
   * @param i2c_dev Linux I2C device node (e.g. "/dev/i2c-1").
   * @param addr    7-bit I2C address (commonly 0x60).
   */
  PCA9685(const std::string& i2c_dev, uint8_t addr);

  /**
   * @brief Initialize the PCA9685 PWM controller.
   *
   * This configures the PWM frequency by temporarily setting SLEEP, updating the
   * prescale register, then waking the device. It also enables auto-increment
   * for efficient multi-register writes.
   *
   * @param pwm_hz Desired PWM frequency in Hz (default: 1600).
   *
   * @note Must be called before setPWM().
   */
  void init(uint16_t pwm_hz = 1600);

  /**
   * @brief Set the PWM on/off counts for a channel.
   *
   * The PCA9685 uses 12-bit counters (0-4095). Common usage for simple duty-cycle
   * PWM is to set @p on = 0 and @p off proportional to duty cycle.
   *
   * @param channel PWM channel index [0..15].
   * @param on      ON count  [0..4095].
   * @param off     OFF count [0..4095].
   */
  void setPWM(uint8_t channel, uint16_t on, uint16_t off);

private:
  I2CDevice i2c_;
};

/**
 * @brief Motor driver using Adafruit MotorHAT-style PCA9685 channel mapping.
 *
 * Each motor is controlled by:
 *  - One PWM enable channel (speed)
 *  - Two direction channels (IN1/IN2)
 *
 * This matches the behavior of Adafruit_MotorHAT used in the NVIDIA JetBot stack.
 *
 * Default channel mapping (Adafruit MotorHAT):
 *  - Motor 1: PWM=8, IN1=10, IN2=9
 *  - Motor 2: PWM=13, IN1=11, IN2=12
 *
 * @ingroup jetbot_base
 */
class MotorHATMotors {
public:
  /**
   * @brief Construct the motor driver.
   *
   * @param i2c_dev Linux I2C device node (e.g. "/dev/i2c-1").
   * @param addr    7-bit I2C address of PCA9685 (commonly 0x60).
   */
  MotorHATMotors(const std::string& i2c_dev, uint8_t addr);

  /**
   * @brief Initialize the PCA9685 and stop all motor outputs.
   *
   * @param pwm_hz PWM frequency in Hz (default: 1600).
   */
  void init(uint16_t pwm_hz = 1600);

  /**
   * @brief Configure channel mapping for a motor.
   *
   * @param motor_index Motor index (1 or 2).
   * @param pwm_channel PCA9685 channel used for speed/enable PWM.
   * @param in1_channel PCA9685 channel used for direction IN1.
   * @param in2_channel PCA9685 channel used for direction IN2.
   *
   * @throws std::runtime_error on invalid motor index.
   */
  void setMotorChannels(int motor_index,
                        uint8_t pwm_channel,
                        uint8_t in1_channel,
                        uint8_t in2_channel);

  /**
   * @brief Command a motor with a normalized speed value.
   *
   * @param motor_index Motor index (1 or 2).
   * @param cmd         Normalized command in [-1, 1].
   *
   * @throws std::runtime_error on invalid motor index.
   */
  void setMotor(int motor_index, double cmd);

  /**
   * @brief Stop both motors by setting PWM=0 and direction pins low.
   */
  void stopAll();

private:
  struct MotorMap {
    uint8_t pwm{0};
    uint8_t in1{0};
    uint8_t in2{0};
  };

  void setPinHigh(uint8_t ch);
  void setPinLow(uint8_t ch);

  PCA9685 pca_;
  MotorMap m1_;
  MotorMap m2_;
};

}  // namespace jetbot_base

#endif  // JETBOT_MOTOR_DRIVER_HPP__