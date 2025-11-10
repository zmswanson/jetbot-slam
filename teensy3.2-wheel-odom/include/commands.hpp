#ifndef JETBOT_ODOM_COMMANDS_HPP_
#define JETBOT_ODOM_COMMANDS_HPP_
/**
 * @file commands.hpp
 * @brief ASCII command channel for Teensy wheel odom firmware.
 *
 * Commands are accepted over the same UART as the telemetry stream.
 * In binary output mode, responses are still ASCII lines; the ROS side should
 * be robust to re-sync after occasional ASCII responses.
 */

#include <Arduino.h>
#include "params.hpp"

namespace jetbot_odom {

/** @brief Output modes. */
enum class OutputMode : uint8_t { JETBOT_BIN = 0, JETBOT_JSON = 1 };

/**
 * @brief Process incoming commands from a stream.
 *
 * @param io Serial stream to read commands from and write responses to.
 * @param params Persistent parameters (may be modified).
 * @param mode Current output mode (may be modified).
 * @param reset_odom Flag set true if odometry counters should reset.
 */
void processCommands(Stream& io, Params& params, OutputMode& mode, bool& reset_odom);

} // namespace jetbot_odom

#endif // JETBOT_ODOM_COMMANDS_HPP_