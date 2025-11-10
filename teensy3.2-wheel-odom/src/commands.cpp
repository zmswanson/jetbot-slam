#include "commands.hpp"
#include "params.hpp"

namespace jetbot_odom {

static String g_line;

static void print_help(Stream& io) {
  io.println(F("# Commands:"));
  io.println(F("#   HELP"));
  io.println(F("#   PING"));
  io.println(F("#   GET_PARAMS"));
  io.println(F("#   SET_PARAM <key> <value>   (keys: counts_per_rev, report_hz, invert_left, invert_right)"));
  io.println(F("#   SAVE_PARAMS"));
  io.println(F("#   LOAD_PARAMS"));
  io.println(F("#   RESET_ODOM"));
  io.println(F("#   MODE BIN|JSON"));
  io.println(F("#   RATE <hz>"));
}

static void print_params(Stream& io, const Params& p, OutputMode mode) {
  io.print(F("# params: counts_per_rev=")); io.print(p.counts_per_rev, 3);
  io.print(F(" report_hz=")); io.print(p.report_hz);
  io.print(F(" invert_left=")); io.print((int)p.invert_left);
  io.print(F(" invert_right=")); io.print((int)p.invert_right);
  io.print(F(" mode=")); io.println(mode == OutputMode::JETBOT_BIN ? F("BIN") : F("JSON"));
}

static bool parse_int(const String& s, int& out) {
  char* end=nullptr;
  long v = strtol(s.c_str(), &end, 10);
  if (!end || *end != '\0') return false;
  out = (int)v;
  return true;
}

static bool parse_float(const String& s, float& out) {
  char* end=nullptr;
  float v = strtof(s.c_str(), &end);
  if (!end || *end != '\0') return false;
  out = v;
  return true;
}

static void handle_line(Stream& io, const String& line, Params& params, OutputMode& mode, bool& reset_odom) {
  // Tokenize by spaces
  String cmd;
  int first_space = line.indexOf(' ');
  if (first_space < 0) cmd = line;
  else cmd = line.substring(0, first_space);

  cmd.toUpperCase();

  if (cmd.length() == 0) return;

  if (cmd == "HELP") { print_help(io); return; }
  if (cmd == "PING") { io.println(F("# PONG")); return; }
  if (cmd == "RESET_ODOM") { reset_odom = true; io.println(F("# OK RESET_ODOM")); return; }

  if (cmd == "GET_PARAMS") { print_params(io, params, mode); return; }

  if (cmd == "LOAD_PARAMS") {
    Params p;
    if (loadParams(p)) { params = p; mode = (params.output_mode==1)?OutputMode::JETBOT_JSON:OutputMode::JETBOT_BIN; io.println(F("# OK LOAD_PARAMS")); }
    else { io.println(F("# ERR LOAD_PARAMS")); }
    return;
  }

  if (cmd == "SAVE_PARAMS") {
    params.output_mode = (mode==OutputMode::JETBOT_JSON)?1:0;
    if (saveParams(params)) io.println(F("# OK SAVE_PARAMS"));
    else io.println(F("# ERR SAVE_PARAMS"));
    return;
  }

  if (cmd == "MODE") {
    if (first_space < 0) { io.println(F("# ERR MODE missing arg")); return; }
    String arg = line.substring(first_space+1); arg.trim(); arg.toUpperCase();
    if (arg == "BIN") { mode = OutputMode::JETBOT_BIN; io.println(F("# OK MODE BIN")); }
    else if (arg == "JSON") { mode = OutputMode::JETBOT_JSON; io.println(F("# OK MODE JSON")); }
    else io.println(F("# ERR MODE arg must be BIN or JSON"));
    return;
  }

  if (cmd == "RATE") {
    if (first_space < 0) { io.println(F("# ERR RATE missing arg")); return; }
    String arg = line.substring(first_space+1); arg.trim();
    int hz=0;
    if (!parse_int(arg, hz) || hz <= 0 || hz > 500) { io.println(F("# ERR RATE invalid")); return; }
    params.report_hz = (uint16_t)hz;
    io.println(F("# OK RATE"));
    return;
  }

  if (cmd == "SET_PARAM") {
    // SET_PARAM <key> <value>
    // crude split
    String rest = (first_space<0) ? "" : line.substring(first_space+1);
    rest.trim();
    int sp = rest.indexOf(' ');
    if (sp < 0) { io.println(F("# ERR SET_PARAM missing key/value")); return; }
    String key = rest.substring(0, sp); key.trim(); key.toLowerCase();
    String val = rest.substring(sp+1); val.trim();

    if (key == "counts_per_rev") {
      float f=0;
      if (!parse_float(val, f) || f < 1.0f) { io.println(F("# ERR counts_per_rev")); return; }
      params.counts_per_rev = f;
      io.println(F("# OK counts_per_rev"));
      return;
    }
    if (key == "invert_left") {
      int v=0;
      if (!parse_int(val, v) || !(v==1 || v==-1)) { io.println(F("# ERR invert_left")); return; }
      params.invert_left = (int8_t)v;
      io.println(F("# OK invert_left"));
      return;
    }
    if (key == "invert_right") {
      int v=0;
      if (!parse_int(val, v) || !(v==1 || v==-1)) { io.println(F("# ERR invert_right")); return; }
      params.invert_right = (int8_t)v;
      io.println(F("# OK invert_right"));
      return;
    }
    if (key == "report_hz") {
      int hz=0;
      if (!parse_int(val, hz) || hz<=0 || hz>500) { io.println(F("# ERR report_hz")); return; }
      params.report_hz = (uint16_t)hz;
      io.println(F("# OK report_hz"));
      return;
    }

    io.println(F("# ERR SET_PARAM unknown key"));
    return;
  }

  io.println(F("# ERR unknown command (HELP)"));
}

void processCommands(Stream& io, Params& params, OutputMode& mode, bool& reset_odom) {
  while (io.available()) {
    char c = (char)io.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String line = g_line;
      g_line = "";
      line.trim();
      if (line.length() > 0) handle_line(io, line, params, mode, reset_odom);
    } else {
      if (g_line.length() < 200) g_line += c;
      else { // overflow, reset
        g_line = "";
      }
    }
  }
}

} // namespace jetbot_odom
