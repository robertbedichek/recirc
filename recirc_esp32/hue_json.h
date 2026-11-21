// hue_json.h: a tiny streaming JSON scanner for the Hue bridge's replies, fed one character at a time so a
// 50 KB reply never has to sit in memory.  Plain C++ (no Arduino), so it can be tested on a desktop:
//   c++ -std=c++17 -o /tmp/t test_hue_json.cpp && /tmp/t
//
// Depth counts the open objects/arrays: the reply's outer object is depth 1.  Callbacks:
//   on_open(ctx, depth, key)          an object or array opened; depth is its own depth, key its name ("" in arrays)
//   on_value(ctx, depth, key, value)  a scalar inside the container at depth; strings come unquoted, others as
//                                     written (87, true, null).  An array's elements come with the array's own
//                                     name as key ("lights": ["1", "2"] gives key "lights" twice)
//   on_close(ctx, depth)              the container at depth closed
// Keys and values longer than 47 characters are cut short.

#pragma once
#include <string.h>
#include <stdint.h>

struct Json_Scan {
  void *ctx = nullptr;
  void (*on_open)(void *ctx, int depth, const char *key) = nullptr;
  void (*on_value)(void *ctx, int depth, const char *key, const char *value) = nullptr;
  void (*on_close)(void *ctx, int depth) = nullptr;

  int depth = 0;
  bool in_str = false, esc = false, want_value = false, have_scalar = false;
  char key[48] = "", tok[48] = "", akey[48] = "";   // akey: the name of the innermost open array
  uint32_t arrays = 0;                               // bit d set: the container at depth d is an array

  bool in_array(void) const { return depth > 0 && depth < 32 && (arrays >> depth & 1); }
  void emit_elem(void) { if (on_value) on_value(ctx, depth, akey, tok); }
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
        else if (in_array()) emit_elem();
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
      if (depth < 32) { if (c == '[') arrays |= 1u << depth; else arrays &= ~(1u << depth); }
      if (c == '[') memcpy(akey, want_value ? key : "", sizeof(akey));
      if (on_open) on_open(ctx, depth, want_value ? key : "");
      want_value = false; key[0] = 0; reset_tok();
      return;
    case '}': case ']':
      if (want_value && have_scalar) emit();
      else if (in_array() && have_scalar) emit_elem();
      have_scalar = false;
      want_value = false;
      if (on_close) on_close(ctx, depth);
      depth--; reset_tok();
      return;
    case ',':
      if (want_value && have_scalar) emit();
      else if (in_array() && have_scalar) emit_elem();
      want_value = false; have_scalar = false; reset_tok();
      return;
    case ' ': case '\n': case '\r': case '\t': return;
    default:
      if (want_value || in_array()) { add(c); have_scalar = true; }
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
  char last[20];        // state.lastupdated: the last button press or sensor event, UTC ("2026-11-18T23:20:01")
  char fw[24];          // swversion, e.g. "2.45.2_hF4400CA"
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
      else if (!strcmp(key, "swversion")) strncpy(s->cur.fw, v, sizeof(s->cur.fw) - 1);
    } else if (depth == 3) {                 // "config": {"on": true, "battery": 87, "reachable": true, ...}
      if (!strcmp(key, "lastupdated") && v[0] >= '0' && v[0] <= '9') strncpy(s->cur.last, v, sizeof(s->cur.last) - 1);
      else if (!strcmp(key, "battery") && v[0] >= '0' && v[0] <= '9') s->cur.battery = atoi_(v);
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

// Lights from GET /api/<key>/lights: who made each one, its model and firmware.
//   {"3": {"state": {"on": true, ..., "reachable": true}, "type": "...", "name": "Kitchen 1", "modelid": "LCA001",
//          "manufacturername": "Signify Netherlands B.V.", "productname": "Hue color lamp", "swversion": "1.122.2", ...}}
struct Hue_Light {
  char id[8];
  char name[32];
  char maker[28];
  char model[16];
  char product[28];
  char fw[24];
  int reachable;        // 1, 0, or -1 if not reported
};

struct Hue_Light_Scan {
  Hue_Light *out;
  int max, count = 0;
  Hue_Light cur;

  static void open(void *ctx, int depth, const char *key)
  {
    Hue_Light_Scan *s = (Hue_Light_Scan *)ctx;
    if (depth != 2) return;
    memset(&s->cur, 0, sizeof(s->cur));
    strncpy(s->cur.id, key, sizeof(s->cur.id) - 1);
    s->cur.reachable = -1;
  }
  static void value(void *ctx, int depth, const char *key, const char *v)
  {
    Hue_Light_Scan *s = (Hue_Light_Scan *)ctx;
    Hue_Light &c = s->cur;
    if (depth == 2) {
      if (!strcmp(key, "name")) strncpy(c.name, v, sizeof(c.name) - 1);
      else if (!strcmp(key, "modelid")) strncpy(c.model, v, sizeof(c.model) - 1);
      else if (!strcmp(key, "manufacturername")) strncpy(c.maker, v, sizeof(c.maker) - 1);
      else if (!strcmp(key, "productname")) strncpy(c.product, v, sizeof(c.product) - 1);
      else if (!strcmp(key, "swversion")) strncpy(c.fw, v, sizeof(c.fw) - 1);
    } else if (depth == 3 && !strcmp(key, "reachable")) {
      c.reachable = !strcmp(v, "true");
    }
  }
  static void close(void *ctx, int depth)
  {
    Hue_Light_Scan *s = (Hue_Light_Scan *)ctx;
    if (depth != 2 || !s->cur.id[0] || s->count >= s->max) return;
    s->out[s->count++] = s->cur;
  }

  void attach(Json_Scan &j, Hue_Light *table, int table_max)
  {
    out = table; max = table_max; count = 0;
    j.ctx = this; j.on_open = open; j.on_value = value; j.on_close = close;
  }
};

// Rooms and zones from GET /api/<key>/groups:
//   {"5": {"name": "Kitchen", "lights": ["5", "6"], "type": "Room", "class": "Kitchen", ...}, ...}
// Calls back once per Room or Zone with its id, name, type and its lights as "5,6,7".
struct Hue_Group_Scan {
  void (*on_group)(void *user, const char *id, const char *name, const char *type, const char *lights) = nullptr;
  void *user = nullptr;
  char id[8], name[48], type[16], lights[400];
  static void open(void *ctx, int depth, const char *key)
  {
    Hue_Group_Scan *g = (Hue_Group_Scan *)ctx;
    if (depth != 2) return;
    strncpy(g->id, key, sizeof(g->id) - 1); g->id[sizeof(g->id) - 1] = 0;
    g->name[0] = g->type[0] = g->lights[0] = 0;
  }
  static void value(void *ctx, int depth, const char *key, const char *v)
  {
    Hue_Group_Scan *g = (Hue_Group_Scan *)ctx;
    if (depth == 2 && !strcmp(key, "name")) { strncpy(g->name, v, sizeof(g->name) - 1); g->name[sizeof(g->name) - 1] = 0; }
    else if (depth == 2 && !strcmp(key, "type")) { strncpy(g->type, v, sizeof(g->type) - 1); g->type[sizeof(g->type) - 1] = 0; }
    else if (depth == 3 && !strcmp(key, "lights")) {
      size_t n = strlen(g->lights);
      if (n + strlen(v) + 2 < sizeof(g->lights)) { if (n) g->lights[n++] = ','; strcpy(g->lights + n, v); }
    }
  }
  static void close(void *ctx, int depth)
  {
    Hue_Group_Scan *g = (Hue_Group_Scan *)ctx;
    if (depth != 2 || !g->on_group) return;
    if (!strcmp(g->type, "Room") || !strcmp(g->type, "Zone")) g->on_group(g->user, g->id, g->name, g->type, g->lights);
  }
  void attach(Json_Scan &j) { j.ctx = this; j.on_open = open; j.on_value = value; j.on_close = close; }
};

// Which groups each switch or sensor acts on, from GET /api/<key>/rules:
//   {"1": {"conditions": [{"address": "/sensors/2/state/buttonevent", ...}], "actions": [{"address": "/groups/5/action", ...}]}}
// Calls back once per rule for every (sensor, group) pair in it.  Switches set up by newer Hue apps may have no
// rules here (they live in the bridge's newer interface), so a switch can come out with no group.
struct Hue_Rule_Scan {
  void (*on_pair)(void *user, const char *sensor, const char *group) = nullptr;
  void *user = nullptr;
  char sensors[4][8], groups[6][8];
  int ns = 0, ng = 0;
  static void take(const char *v, const char *prefix, char (*tab)[8], int &n, int max)
  {
    size_t pl = strlen(prefix);
    if (strncmp(v, prefix, pl) || n >= max) return;
    int k = 0;
    while (v[pl + k] >= '0' && v[pl + k] <= '9' && k < 7) { tab[n][k] = v[pl + k]; k++; }
    tab[n][k] = 0;
    if (!k) return;
    for (int i = 0; i < n; i++) if (!strcmp(tab[i], tab[n])) return;
    n++;
  }
  bool in_conditions = false;               // inside "conditions" (else "actions")
  static void open(void *ctx, int depth, const char *key)
  {
    Hue_Rule_Scan *r = (Hue_Rule_Scan *)ctx;
    if (depth == 2) r->ns = r->ng = 0;
    else if (depth == 3) r->in_conditions = !strcmp(key, "conditions");
  }
  static void value(void *ctx, int depth, const char *key, const char *v)
  {
    Hue_Rule_Scan *r = (Hue_Rule_Scan *)ctx;
    if (depth != 4 || strcmp(key, "address")) return;
    if (r->in_conditions) take(v, "/sensors/", r->sensors, r->ns, 4);   // the switch or sensor that triggers it
    else take(v, "/groups/", r->groups, r->ng, 6);                     // the rooms it acts on
  }
  static void close(void *ctx, int depth)
  {
    Hue_Rule_Scan *r = (Hue_Rule_Scan *)ctx;
    if (depth != 2 || !r->on_pair) return;
    for (int i = 0; i < r->ns; i++) for (int k = 0; k < r->ng; k++) r->on_pair(r->user, r->sensors[i], r->groups[k]);
  }
  void attach(Json_Scan &j) { j.ctx = this; j.on_open = open; j.on_value = value; j.on_close = close; }
};

// From GET /api/<key>/config: the Zigbee channel, and the bridge's own firmware and update settings:
//   "modelid", "swversion", "apiversion", "swupdate2": {"state", "bridge": {"lastinstall"}, "autoinstall": {"on"}}
struct Hue_Config_Scan {
  int zigbee_channel = -1;
  int autoinstall = -1;                    // 1 = automatic updates on, 0 = off, -1 = not in the reply
  char model[16] = "", sw[24] = "", api[16] = "", update_state[24] = "", last_install[24] = "";
  char sect2[24] = "", sect3[24] = "";     // names of the open objects at depth 2 and 3
  static void copy(char *dst, size_t n, const char *v) { strncpy(dst, v, n - 1); dst[n - 1] = 0; }
  static void open(void *ctx, int depth, const char *key)
  {
    Hue_Config_Scan *c = (Hue_Config_Scan *)ctx;
    if (depth == 2) { copy(c->sect2, sizeof(c->sect2), key); c->sect3[0] = 0; }
    else if (depth == 3) copy(c->sect3, sizeof(c->sect3), key);
  }
  static void close(void *ctx, int depth)
  {
    Hue_Config_Scan *c = (Hue_Config_Scan *)ctx;
    if (depth == 2) c->sect2[0] = 0;
    else if (depth == 3) c->sect3[0] = 0;
  }
  static void value(void *ctx, int depth, const char *key, const char *v)
  {
    Hue_Config_Scan *c = (Hue_Config_Scan *)ctx;
    if (depth == 1) {
      if (!strcmp(key, "zigbeechannel")) c->zigbee_channel = Hue_Sensor_Scan::atoi_(v);
      else if (!strcmp(key, "modelid")) copy(c->model, sizeof(c->model), v);
      else if (!strcmp(key, "swversion")) copy(c->sw, sizeof(c->sw), v);
      else if (!strcmp(key, "apiversion")) copy(c->api, sizeof(c->api), v);
    } else if (!strcmp(c->sect2, "swupdate2")) {
      if (depth == 2 && !strcmp(key, "state")) copy(c->update_state, sizeof(c->update_state), v);
      else if (depth == 3 && !strcmp(c->sect3, "bridge") && !strcmp(key, "lastinstall"))
        copy(c->last_install, sizeof(c->last_install), v);
      else if (depth == 3 && !strcmp(c->sect3, "autoinstall") && !strcmp(key, "on"))
        c->autoinstall = !strcmp(v, "true");
    }
  }
  void attach(Json_Scan &j) { j.ctx = this; j.on_value = value; j.on_open = open; j.on_close = close; }
};

#ifdef ARDUINO
// One paired bridge (recirc_esp32.ino).  Here rather than in the sketch so the Arduino build's generated
// prototypes, which go above the sketch's first function, can name it.
struct Hue_Bridge {
  String ip, key;
  uint32_t fingerprint;
  bool have_fingerprint, error_reported;
  int lights, lights_on, zigbee_channel;
  String sw;                               // bridge firmware at the last reading since boot, "" if none yet
  int autoinstall;                         // automatic updates at the last reading: 1 on, 0 off, -1 unknown
};
#endif
