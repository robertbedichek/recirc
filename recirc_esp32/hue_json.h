// hue_json.h: a tiny streaming JSON scanner for the Hue bridge's replies, fed one character at a time so a
// 50 KB reply never has to sit in memory.  Plain C++ (no Arduino), so it can be tested on a desktop:
//   c++ -std=c++17 -o /tmp/t test_hue_json.cpp && /tmp/t
//
// Depth counts the open objects/arrays: the reply's outer object is depth 1.  Callbacks:
//   on_open(ctx, depth, key)          an object or array opened; depth is its own depth, key its name ("" in arrays)
//   on_value(ctx, depth, key, value)  a scalar inside the container at depth; strings come unquoted, others as
//                                     written (87, true, null)
//   on_close(ctx, depth)              the container at depth closed
// Keys and values longer than 47 characters are cut short.

#pragma once
#include <string.h>

struct Json_Scan {
  void *ctx = nullptr;
  void (*on_open)(void *ctx, int depth, const char *key) = nullptr;
  void (*on_value)(void *ctx, int depth, const char *key, const char *value) = nullptr;
  void (*on_close)(void *ctx, int depth) = nullptr;

  int depth = 0;
  bool in_str = false, esc = false, want_value = false, have_scalar = false;
  char key[48] = "", tok[48] = "";
  int tok_len = 0;

  void reset_tok(void) { tok_len = 0; tok[0] = 0; }
  void add(char c) { if (tok_len < (int)sizeof(tok) - 1) { tok[tok_len++] = c; tok[tok_len] = 0; } }
  void emit(void) { if (on_value) on_value(ctx, depth, key, tok); }

  void feed(char c)
  {
    if (in_str) {
      if (esc) { add(c); esc = false; }
      else if (c == '\\') esc = true;
      else if (c == '"') {
        in_str = false;
        if (want_value) { emit(); want_value = false; }
        else { memcpy(key, tok, sizeof(key)); }
      }
      else add(c);
      return;
    }
    switch (c) {
    case '"': in_str = true; reset_tok(); return;
    case ':': want_value = true; have_scalar = false; reset_tok(); return;
    case '{': case '[':
      depth++;
      if (on_open) on_open(ctx, depth, want_value ? key : "");
      want_value = false; key[0] = 0; reset_tok();
      return;
    case '}': case ']':
      if (want_value && have_scalar) emit();
      want_value = false;
      if (on_close) on_close(ctx, depth);
      depth--; reset_tok();
      return;
    case ',':
      if (want_value && have_scalar) emit();
      want_value = false; reset_tok();
      return;
    case ' ': case '\n': case '\r': case '\t': return;
    default:
      if (want_value) { add(c); have_scalar = true; }
      return;
    }
  }
};

// Battery-powered Hue accessories from GET /api/<key>/sensors.  One physical motion sensor shows up as three
// sensors (presence, temperature, light level) with the same battery; only the presence one is kept.
struct Hue_Sensor {
  char id[8];
  char name[48];
  char model[16];
  int battery;          // Percent, -1 if the bridge reports none
  int reachable;        // 1, 0, or -1 if not reported
};

struct Hue_Sensor_Scan {
  Hue_Sensor *out;
  int max, count = 0;
  Hue_Sensor cur;
  char type[24];

  static void open(void *ctx, int depth, const char *key)
  {
    Hue_Sensor_Scan *s = (Hue_Sensor_Scan *)ctx;
    if (depth != 2) return;
    memset(&s->cur, 0, sizeof(s->cur));
    strncpy(s->cur.id, key, sizeof(s->cur.id) - 1);
    s->cur.battery = s->cur.reachable = -1;
    s->type[0] = 0;
  }
  static void value(void *ctx, int depth, const char *key, const char *v)
  {
    Hue_Sensor_Scan *s = (Hue_Sensor_Scan *)ctx;
    if (depth == 2) {
      if (!strcmp(key, "name")) strncpy(s->cur.name, v, sizeof(s->cur.name) - 1);
      else if (!strcmp(key, "modelid")) strncpy(s->cur.model, v, sizeof(s->cur.model) - 1);
      else if (!strcmp(key, "type")) strncpy(s->type, v, sizeof(s->type) - 1);
    } else if (depth == 3) {                 // "config": {"on": true, "battery": 87, "reachable": true, ...}
      if (!strcmp(key, "battery") && v[0] >= '0' && v[0] <= '9') s->cur.battery = atoi_(v);
      else if (!strcmp(key, "reachable")) s->cur.reachable = !strcmp(v, "true");
    }
  }
  static void close(void *ctx, int depth)
  {
    Hue_Sensor_Scan *s = (Hue_Sensor_Scan *)ctx;
    if (depth != 2 || s->cur.battery < 0 || s->count >= s->max) return;
    if (!strcmp(s->type, "ZLLTemperature") || !strcmp(s->type, "ZLLLightLevel")) return;
    s->out[s->count++] = s->cur;
  }
  static int atoi_(const char *v) { int n = 0; while (*v >= '0' && *v <= '9') n = n * 10 + (*v++ - '0'); return n; }

  void attach(Json_Scan &j, Hue_Sensor *table, int table_max)
  {
    out = table; max = table_max; count = 0;
    j.ctx = this; j.on_open = open; j.on_value = value; j.on_close = close;
  }
};

// "zigbeechannel" from GET /api/<key>/config.
struct Hue_Config_Scan {
  int zigbee_channel = -1;
  static void value(void *ctx, int depth, const char *key, const char *v)
  {
    if (depth == 1 && !strcmp(key, "zigbeechannel")) ((Hue_Config_Scan *)ctx)->zigbee_channel = Hue_Sensor_Scan::atoi_(v);
  }
  void attach(Json_Scan &j) { j.ctx = this; j.on_value = value; j.on_open = nullptr; j.on_close = nullptr; }
};

#ifdef ARDUINO
// One paired bridge (recirc_esp32.ino).  Here rather than in the sketch so the Arduino build's generated
// prototypes, which go above the sketch's first function, can name it.
struct Hue_Bridge {
  String ip, key;
  uint32_t fingerprint;
  bool have_fingerprint, error_reported;
  int lights, lights_on, zigbee_channel;
};
#endif
