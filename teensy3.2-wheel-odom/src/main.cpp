#include <Arduino.h>

#include "WheelEncoders.h"
#include "protocol.hpp"
#include "crc16.hpp"
#include "params.hpp"
#include "commands.hpp"

using jetbot_odom::OdomDeltaFrame;
using jetbot_odom::MsgType;
using jetbot_odom::kSyncWord;
using jetbot_odom::kProtoVersion;
using jetbot_odom::crc16_ccitt_false;
using jetbot_odom::Params;
using jetbot_odom::OutputMode;

static constexpr uint8_t LEFT_A  = 2;
static constexpr uint8_t LEFT_B  = 3;
static constexpr uint8_t RIGHT_A = 4;
static constexpr uint8_t RIGHT_B = 5;

WheelEncoders encoders(LEFT_A, LEFT_B, RIGHT_A, RIGHT_B);

static Params g_params;
static OutputMode g_mode = OutputMode::JETBOT_BIN;

static uint32_t g_seq = 0;
static bool g_reset_odom = false;

static void send_json(Stream& io, uint32_t seq, uint32_t dt_us, int32_t dl, int32_t dr) {
  io.print("{\"seq\":"); io.print(seq);
  io.print(",\"dt_us\":"); io.print(dt_us);
  io.print(",\"dl_ticks\":"); io.print(dl);
  io.print(",\"dr_ticks\":"); io.print(dr);
  io.println("}");
}

static void send_bin(Stream& io, uint32_t seq, uint32_t dt_us, int32_t dl, int32_t dr) {
  OdomDeltaFrame f{};
  f.sync = kSyncWord;
  f.version = kProtoVersion;
  f.msg_type = static_cast<uint8_t>(MsgType::ODOM_DELTA);
  f.seq = seq;
  f.dt_us = dt_us;
  f.dl_ticks = dl;
  f.dr_ticks = dr;
  f.crc16 = crc16_ccitt_false(reinterpret_cast<const uint8_t*>(&f), offsetof(OdomDeltaFrame, crc16));
  io.write(reinterpret_cast<const uint8_t*>(&f), sizeof(f));
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial1.begin(115200);

  // Load params; if invalid, use defaults and save.
  jetbot_odom::defaultParams(g_params);
  Params loaded;
  if (jetbot_odom::loadParams(loaded)) {
    g_params = loaded;
  } else {
    jetbot_odom::saveParams(g_params);
  }
  g_mode = (g_params.output_mode == 1) ? OutputMode::JETBOT_JSON : OutputMode::JETBOT_BIN;

  encoders.begin();

  // Optional startup message (ASCII). ROS binary parser will resync quickly.
  Serial1.println("# JetBot Wheel Odom Teensy starting (type HELP)");
}

void loop() {
  // Command channel (same UART).
  jetbot_odom::processCommands(Serial1, g_params, g_mode, g_reset_odom);
  if (g_reset_odom) {
    encoders.reset();
    g_reset_odom = false;
  }

  const uint32_t period_us = (g_params.report_hz > 0) ? (1000000UL / g_params.report_hz) : 20000UL;

  if (encoders.update(period_us, g_params.invert_left, g_params.invert_right)) {
    const int32_t dl = encoders.dlTicks();
    const int32_t dr = encoders.drTicks();
    const uint32_t dt_us = encoders.dtUs();

    const uint32_t seq = g_seq++;
    if (g_mode == OutputMode::JETBOT_JSON) {
      send_json(Serial1, seq, dt_us, dl, dr);
    } else {
      send_bin(Serial1, seq, dt_us, dl, dr);
    }

    // Blink heartbeat every ~1s depending on report rate.
    static uint32_t blink_count = 0;
    if ((++blink_count % g_params.report_hz) == 0) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}
