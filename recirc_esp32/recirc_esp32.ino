// recirc_esp32: hot-water recirculation pump controller for the house (Rob Bedichek, started 2026-11-10).
//
// Board: Waveshare ESP32-S3-Relay-1CH (ESP32-S3R8, 2.4 GHz Wi-Fi, one 10 A relay, PCF85063 RTC, isolated RS485).
// The pump (Grundfos, 120 VAC, about 30 W) is switched by the onboard relay: line through COM/NO.
//
// What it does:
//   Every interval_min minutes, on the clock (:00, :20, :40 by default), if someone is home and it is not quiet
//   hours (23:00-07:00), run the pump until the return line has warmed rise_F above where it started, or for
//   max_run_s seconds, whichever comes first.  The "run now" button (IO2) runs it regardless of presence or the
//   hour.  The pump never runs without a working return-line temperature.
//
//   Someone is home when a family phone has been seen on the Wi-Fi (ARP sweep of the local /24 for their MACs)
//   or a Hue light has changed (polled from the bridge's local API) within the last hold_min minutes.  With
//   neither source configured it assumes someone is home.  "mode home|away" overrides.
//
// Hardware:
//   IO47  relay (high = closed = pump on)
//   IO1   DS18B20 data (4-pin SH1.0 header, yellow lead), 4.7k to 3V3 (red); GND is black
//   IO2   run-now button to GND (orange lead), internal pull-up
//   IO39/IO38  I2C SDA/SCL to the PCF85063 RTC (0x51), kept in UTC
//
// Network:
//   http://recirc/status   JSON state            http://recirc/log   recent output
//   POST /cmd    (X-Token) a console command      POST /update (X-Token) load firmware from the LAN
//   Telemetry lines go to https://bedichek.org/home/api/ingest/recirc ("upload on").
//   Firmware updates are pulled: every hour it reads FW_BASE/recirc.json and, if the build differs, downloads
//   FW_BASE/recirc.bin, checks its SHA-256 and installs it.  A new image that can't reach bedichek.org within a
//   minute rolls back to the previous one.  This works from behind the house's NAT with no VPN.
//
// Console commands (USB serial, or POST /cmd):
//   run | stop                      run the pump now (subject to max_run_s) | stop it
//   mode auto|home|away             presence from phones and Hue | always home | never on schedule
//   set interval|maxrun|rise|hold|quiet <value>   (quiet takes "23-7"; "set" alone lists them)
//   phone add <mac> <name> | phone del <mac> | phones
//   hue pair <ip> (press the bridge's link button first) | hue forget
//   upload on|off | update check | reboot | help

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <NetworkClientSecure.h>
#include <Update.h>
#include <Preferences.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <esp_task_wdt.h>
#include <esp_ota_ops.h>
#include <esp_sntp.h>
#include <mbedtls/sha256.h>
#include "lwip/etharp.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"

#include "secrets.h"   // RC_WIFI_SSID, RC_WIFI_PASS (optional RC_WIFI2_SSID, RC_WIFI2_PASS), RC_LAN_TOKEN, RC_INGEST_TOKEN, RC_FW_TOKEN: written by rcctl, not in git
#include "version.h"   // RC_BUILD_ID: written by rcctl on every build, not in git

#define HOSTNAME        "recirc"
#define RELAY_PIN       (47)
#define ONEWIRE_PIN     (1)
#define BUTTON_PIN      (2)
#define I2C_SDA_PIN     (39)
#define I2C_SCL_PIN     (38)
#define PCF85063_ADDR   (0x51)
#define SERIAL_BAUD     (115200)
#define TIMEZONE        "PST8PDT,M3.2.0,M11.1.0"
#define INGEST_URL      "https://bedichek.org/home/api/ingest/recirc"
#define FW_BASE         "https://bedichek.org/home/api/fw/recirc/"

// ---------------------------------------------------------------------------------------------------------------
// Settings, kept in flash ("set ...", "mode ...", "phone ...", "hue ...")

struct Settings {
  int interval_min = 20;      // Minutes between scheduled runs, aligned to the hour
  int max_run_s = 80;         // Longest a run may last
  float rise_F = 4.0;         // Stop once the return line is this much warmer than at the start
  int hold_min = 60;          // Someone is home for this long after a phone or Hue light was last seen
  int quiet_start_h = 23;     // No scheduled runs from quiet_start_h:00 ...
  int quiet_end_h = 7;        // ... to quiet_end_h:00, local time
  char mode = 'a';            // 'a' auto (presence), 'h' always home, 'w' away (no scheduled runs)
  bool upload = false;        // Send telemetry to bedichek.org
  String phones;              // "aa:bb:cc:dd:ee:ff=name;..." lower case
  String hue_ip;
  String hue_user;            // The bridge's API key from "hue pair"
} cfg;

Preferences prefs;

void save_settings(void)
{
  prefs.begin("recirc", false);
  prefs.putInt("interval", cfg.interval_min);
  prefs.putInt("maxrun", cfg.max_run_s);
  prefs.putFloat("rise", cfg.rise_F);
  prefs.putInt("hold", cfg.hold_min);
  prefs.putInt("qstart", cfg.quiet_start_h);
  prefs.putInt("qend", cfg.quiet_end_h);
  prefs.putChar("mode", cfg.mode);
  prefs.putBool("upload", cfg.upload);
  prefs.putString("phones", cfg.phones);
  prefs.putString("hue_ip", cfg.hue_ip);
  prefs.putString("hue_user", cfg.hue_user);
  prefs.end();
}

void load_settings(void)
{
  prefs.begin("recirc", true);
  cfg.interval_min = prefs.getInt("interval", cfg.interval_min);
  cfg.max_run_s = prefs.getInt("maxrun", cfg.max_run_s);
  cfg.rise_F = prefs.getFloat("rise", cfg.rise_F);
  cfg.hold_min = prefs.getInt("hold", cfg.hold_min);
  cfg.quiet_start_h = prefs.getInt("qstart", cfg.quiet_start_h);
  cfg.quiet_end_h = prefs.getInt("qend", cfg.quiet_end_h);
  cfg.mode = prefs.getChar("mode", cfg.mode);
  cfg.upload = prefs.getBool("upload", cfg.upload);
  cfg.phones = prefs.getString("phones", "");
  cfg.hue_ip = prefs.getString("hue_ip", "");
  cfg.hue_user = prefs.getString("hue_user", "");
  prefs.end();
}

// ---------------------------------------------------------------------------------------------------------------
// Output: every line goes to USB serial, a 32 KB ring for /log and, with "upload on", a queue for bedichek.org.

class Line_Buffer {
public:
  void init(size_t capacity) { buf = (char *)(psramFound() ? ps_malloc(capacity) : malloc(capacity)); cap = buf ? capacity : 0; }
  size_t length(void) { return len; }
  void append(const char *line, size_t n)
  {
    if (n > cap) return;
    if (len + n > cap) {                      // Drop the oldest whole lines to make room
      size_t drop = len + n - cap;
      while (drop < len && buf[drop - 1] != '\n') drop++;
      consume(drop);
    }
    memcpy(buf + len, line, n);
    len += n;
  }
  size_t peek(char *dst, size_t max)          // Whole lines from the front, up to max bytes
  {
    size_t n = len < max ? len : max;
    while (n > 0 && buf[n - 1] != '\n') n--;
    memcpy(dst, buf, n);
    return n;
  }
  void consume(size_t n) { if (n >= len) { len = 0; } else { memmove(buf, buf + n, len - n); len -= n; } }
  void copy_tail(String &dst, size_t max)
  {
    size_t start = len > max ? len - max : 0;
    while (start > 0 && start < len && buf[start - 1] != '\n') start++;
    dst.concat(buf + start, len - start);
  }
private:
  char *buf = nullptr;
  size_t cap = 0, len = 0;
};

Line_Buffer recent_lines, upload_queue;
SemaphoreHandle_t lines_mutex;

class Tee_Output : public Print {
public:
  size_t write(uint8_t c) override
  {
    Serial.write(c);
    if (c == '\r') return 1;
    if (n < sizeof(line) - 1) line[n++] = c;
    if (c == '\n') {
      if (line[n - 1] != '\n') line[n++] = '\n';
      xSemaphoreTake(lines_mutex, portMAX_DELAY);
      recent_lines.append(line, n);
      if (cfg.upload) upload_queue.append(line, n);
      xSemaphoreGive(lines_mutex);
      n = 0;
    }
    return 1;
  }
  size_t write(const uint8_t *b, size_t len) override { for (size_t i = 0; i < len; i++) write(b[i]); return len; }
private:
  char line[240];
  size_t n = 0;
};

Tee_Output out;

// ---------------------------------------------------------------------------------------------------------------
// Time: NTP when Wi-Fi is up, the PCF85063 (UTC) across power cuts and outages.

bool time_valid = false;
volatile bool ntp_synced = false;

static uint8_t bcd2bin(uint8_t v) { return (v >> 4) * 10 + (v & 0x0f); }
static uint8_t bin2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

bool rtc_read(time_t *t)
{
  Wire.beginTransmission(PCF85063_ADDR);
  Wire.write(0x04);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(PCF85063_ADDR, 7) != 7) return false;
  uint8_t r[7];
  for (int i = 0; i < 7; i++) r[i] = Wire.read();
  if (r[0] & 0x80) return false;              // Oscillator stopped: the time can't be trusted
  struct tm tm = {};
  tm.tm_sec = bcd2bin(r[0] & 0x7f);
  tm.tm_min = bcd2bin(r[1] & 0x7f);
  tm.tm_hour = bcd2bin(r[2] & 0x3f);
  tm.tm_mday = bcd2bin(r[3] & 0x3f);
  tm.tm_mon = bcd2bin(r[5] & 0x1f) - 1;
  tm.tm_year = bcd2bin(r[6]) + 100;           // Register holds year - 2000
  if (tm.tm_year < 125) return false;         // Before 2025: never set
  // mktime() works in local time; the RTC holds UTC, so convert with TZ temporarily set to UTC
  setenv("TZ", "UTC0", 1); tzset();
  time_t v = mktime(&tm);
  setenv("TZ", TIMEZONE, 1); tzset();
  *t = v;
  return true;
}

bool rtc_write(time_t t)
{
  struct tm tm;
  gmtime_r(&t, &tm);
  Wire.beginTransmission(PCF85063_ADDR);
  Wire.write(0x04);
  Wire.write(bin2bcd(tm.tm_sec));             // Clears the oscillator-stop flag
  Wire.write(bin2bcd(tm.tm_min));
  Wire.write(bin2bcd(tm.tm_hour));
  Wire.write(bin2bcd(tm.tm_mday));
  Wire.write(bin2bcd(tm.tm_wday));
  Wire.write(bin2bcd(tm.tm_mon + 1));
  Wire.write(bin2bcd(tm.tm_year - 100));
  return Wire.endTransmission() == 0;
}

void time_sync_notification(struct timeval *tv) { ntp_synced = true; }

void setup_time(void)
{
  setenv("TZ", TIMEZONE, 1);
  tzset();
  time_t t;
  if (rtc_read(&t)) {
    struct timeval tv = {t, 0};
    settimeofday(&tv, nullptr);
    time_valid = true;
  }
  sntp_set_time_sync_notification_cb(time_sync_notification);
  configTzTime(TIMEZONE, "pool.ntp.org", "time.apple.com");
}

String timestamp(void)            // "YYYY-MM-DD HH:MM:SS", local; the ingest service wants data lines to start this way
{
  char b[24];
  time_t t = time(nullptr);
  struct tm tm;
  localtime_r(&t, &tm);
  strftime(b, sizeof(b), "%Y-%m-%d %H:%M:%S", &tm);
  return String(b);
}

void log_event(const String &text)    // A '#' line, with the time when it is known
{
  if (time_valid) out.println("# " + timestamp() + " " + text);
  else out.println("# " + text);
}

// ---------------------------------------------------------------------------------------------------------------
// Return-line temperature: one DS18B20 at 11 bits (0.125 C, 375 ms), read without blocking about twice a second.

OneWire one_wire(ONEWIRE_PIN);
DallasTemperature probes(&one_wire);
DeviceAddress probe_addr;
bool probe_found = false;
float return_F = NAN;                 // Last good reading
unsigned long return_ms = 0;          // millis() of the last good reading, 0 if none
unsigned long probe_errors = 0;       // Bad readings since boot
unsigned long conversion_started = 0;

String rom_text(const DeviceAddress a)
{
  char b[24];
  snprintf(b, sizeof(b), "%02x%02x%02x%02x%02x%02x%02x%02x", a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]);
  return String(b);
}

void find_probe(void)
{
  probes.begin();
  probe_found = probes.getAddress(probe_addr, 0);
  if (probe_found) {
    probes.setResolution(probe_addr, 11);
    probes.setWaitForConversion(false);
    log_event("return-line probe " + rom_text(probe_addr) + (probes.getDeviceCount() > 1 ? " (first of several)" : ""));
  }
}

bool probe_ok(void) { return return_ms != 0 && millis() - return_ms < 5000; }

unsigned long probe_alert_ms;     // When "no probe" was last alerted, 0 once the probe is back

void read_probe(void)             // Called every 100 ms
{
  static unsigned long last_search;
  if (!probe_found) {
    if (millis() - last_search > 30000) {
      last_search = millis();
      find_probe();
      if (!probe_found && (probe_alert_ms == 0 || millis() - probe_alert_ms > 3600000UL)) {   // Then hourly, not every 30 s
        probe_alert_ms = millis();
        log_event("alert no return-line probe on the 1-Wire bus");
      }
      if (probe_found) probe_alert_ms = 0;
    }
    return;
  }
  if (conversion_started == 0) {
    probes.requestTemperaturesByAddress(probe_addr);
    conversion_started = millis();
    return;
  }
  if (millis() - conversion_started < 400) return;
  conversion_started = 0;
  float c = probes.getTempC(probe_addr);      // Checks the scratchpad CRC; -127 if the probe didn't answer
  float f = c * 9.0 / 5.0 + 32.0;
  bool plausible = c > -20 && c < 100;
  if (c == 85.0 && (isnan(return_F) || fabs(f - return_F) > 10)) plausible = false;  // Power-on value, not a reading
  if (plausible) {
    return_F = f;
    return_ms = millis();
  } else {
    probe_errors++;
    if (millis() - return_ms > 30000) {       // Persistently bad: look for the probe again
      probe_found = false;
      last_search = 0;
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Presence: family phones by ARP, Hue lights by their state changing.

struct Phone { uint8_t mac[6]; String name; unsigned long seen_ms; uint32_t ip; };
Phone phones[12];
int phone_count = 0;
SemaphoreHandle_t presence_mutex;
unsigned long hue_change_ms = 0;      // millis() of the last Hue light change, 0 if none since boot
int hue_lights = -1, hue_lights_on = -1;
bool hue_error_reported = false;

bool parse_mac(const String &s, uint8_t mac[6])
{
  unsigned v[6];
  if (sscanf(s.c_str(), "%x:%x:%x:%x:%x:%x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) return false;
  for (int i = 0; i < 6; i++) { if (v[i] > 255) return false; mac[i] = v[i]; }
  return true;
}

String mac_text(const uint8_t m[6])
{
  char b[20];
  snprintf(b, sizeof(b), "%02x:%02x:%02x:%02x:%02x:%02x", m[0], m[1], m[2], m[3], m[4], m[5]);
  return String(b);
}

void load_phones(void)            // From cfg.phones; keeps the last-seen times of phones still listed
{
  xSemaphoreTake(presence_mutex, portMAX_DELAY);
  Phone old[12];
  int old_count = phone_count;
  for (int i = 0; i < old_count; i++) old[i] = phones[i];
  phone_count = 0;
  int start = 0;
  while (start < (int)cfg.phones.length() && phone_count < 12) {
    int end = cfg.phones.indexOf(';', start);
    if (end < 0) end = cfg.phones.length();
    String item = cfg.phones.substring(start, end);
    int eq = item.indexOf('=');
    Phone p = {};
    if (eq > 0 && parse_mac(item.substring(0, eq), p.mac)) {
      p.name = item.substring(eq + 1);
      for (int i = 0; i < old_count; i++) {
        if (memcmp(old[i].mac, p.mac, 6) == 0) { p.seen_ms = old[i].seen_ms; p.ip = old[i].ip; }
      }
      phones[phone_count++] = p;
    }
    start = end + 1;
  }
  xSemaphoreGive(presence_mutex);
}

unsigned long last_phone_ms(void)     // millis() a family phone was last seen, 0 if never
{
  unsigned long best = 0;
  xSemaphoreTake(presence_mutex, portMAX_DELAY);
  for (int i = 0; i < phone_count; i++) {
    if (phones[i].seen_ms != 0 && (best == 0 || (long)(phones[i].seen_ms - best) > 0)) best = phones[i].seen_ms;
  }
  xSemaphoreGive(presence_mutex);
  return best;
}

static bool recent(unsigned long ms, int minutes) { return ms != 0 && millis() - ms < (unsigned long)minutes * 60000UL; }

bool someone_home(void)
{
  if (cfg.mode == 'h') return true;
  if (cfg.mode == 'w') return false;
  bool have_phones = phone_count > 0, have_hue = cfg.hue_user.length() > 0;
  if (!have_phones && !have_hue) return true;            // Nothing to go on: assume yes
  return recent(last_phone_ms(), cfg.hold_min) || recent(hue_change_ms, cfg.hold_min);
}

// lwIP's raw ARP calls must run in its own thread
struct Arp_Job { uint32_t first, count; SemaphoreHandle_t done; };

static void arp_request_batch(void *arg)
{
  Arp_Job *job = (Arp_Job *)arg;
  struct netif *nif = netif_default;
  if (nif != nullptr && netif_is_up(nif)) {
    for (uint32_t i = 0; i < job->count; i++) {
      ip4_addr_t a;
      a.addr = htonl(job->first + i);
      etharp_request(nif, &a);
    }
  }
  xSemaphoreGive(job->done);
}

static void arp_scan_table(void *arg)
{
  Arp_Job *job = (Arp_Job *)arg;
  for (size_t i = 0; i < ARP_TABLE_SIZE; i++) {
    ip4_addr_t *ip;
    struct netif *nif;
    struct eth_addr *eth;
    if (etharp_get_entry(i, &ip, &nif, &eth) != 1) continue;
    for (int p = 0; p < phone_count; p++) {
      if (memcmp(phones[p].mac, eth->addr, 6) == 0) {
        phones[p].seen_ms = millis();
        phones[p].ip = ip->addr;
      }
    }
  }
  xSemaphoreGive(job->done);
}

void phone_sweep(void)            // ARP every address in our /24, a few at a time so the small ARP table keeps up
{
  if (phone_count == 0 || WiFi.status() != WL_CONNECTED) return;
  uint32_t me = ntohl((uint32_t)WiFi.localIP());
  uint32_t mask = ntohl((uint32_t)WiFi.subnetMask());
  if (mask != 0xffffff00) mask = 0xffffff00;  // Only ever sweep a /24
  uint32_t net = me & mask;
  Arp_Job job = {0, 0, xSemaphoreCreateBinary()};
  for (uint32_t host = 1; host < 255; host += 6) {
    job.first = net + host;
    job.count = host + 6 <= 255 ? 6 : 255 - host;
    if (tcpip_callback(arp_request_batch, &job) != ERR_OK) break;   // lwIP's queue is full; try next sweep
    xSemaphoreTake(job.done, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(300));
    xSemaphoreTake(presence_mutex, portMAX_DELAY);
    bool queued = tcpip_callback(arp_scan_table, &job) == ERR_OK;
    if (queued) xSemaphoreTake(job.done, portMAX_DELAY);
    xSemaphoreGive(presence_mutex);
    if (!queued) break;
  }
  vSemaphoreDelete(job.done);
}

// Hue: GET /api/<key>/lights and fingerprint every light's "on" and "bri".  A changed fingerprint means someone
// switched or dimmed a light.  Streams the reply rather than parsing it whole, since big houses send 50 KB+.
uint32_t hue_fingerprint = 0;
bool hue_have_fingerprint = false;

void hue_poll(void)
{
  if (cfg.hue_user.length() == 0 || WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  http.setTimeout(8000);
  if (!http.begin("http://" + cfg.hue_ip + "/api/" + cfg.hue_user + "/lights")) return;
  int code = http.GET();
  if (code != 200) {
    if (!hue_error_reported) { log_event("alert Hue bridge at " + cfg.hue_ip + " answered " + String(code)); hue_error_reported = true; }
    http.end();
    return;
  }
  NetworkClient *s = http.getStreamPtr();
  uint32_t h = 2166136261u;                   // FNV-1a over the values that follow "on": and "bri":
  int lights = 0, on = 0;
  char win[7] = {0};                          // The last six characters seen
  int capture = 0;                            // 1 after "on":, 2 after "bri":
  bool saw_error = false;
  unsigned long deadline = millis() + 8000;
  int remaining = http.getSize();             // -1 when chunked
  while (http.connected() && millis() < deadline && remaining != 0) {
    int c = s->read();
    if (c < 0) { delay(2); continue; }
    if (remaining > 0) remaining--;
    memmove(win, win + 1, 5);
    win[5] = (char)c;
    if (capture) {
      if (c == ',' || c == '}') { capture = 0; h = (h ^ '|') * 16777619u; continue; }
      h = (h ^ (uint8_t)c) * 16777619u;
      if (capture == 1 && c == 't') on++;     // "on":true
      continue;
    }
    if (memcmp(win + 1, "\"on\":", 5) == 0) { capture = 1; lights++; }
    else if (memcmp(win, "\"bri\":", 6) == 0) capture = 2;
    else if (memcmp(win, "error\"", 6) == 0) saw_error = true;
  }
  http.end();
  if (saw_error && lights == 0) {
    if (!hue_error_reported) { log_event("alert the Hue bridge rejected our key; run \"hue pair\" again"); hue_error_reported = true; }
    return;
  }
  hue_error_reported = false;
  if (hue_have_fingerprint && h != hue_fingerprint) {
    hue_change_ms = millis();
    log_event("hue change: " + String(on) + " of " + String(lights) + " lights on");
  }
  hue_fingerprint = h;
  hue_have_fingerprint = true;
  hue_lights = lights;
  hue_lights_on = on;
}

String hue_pair(const String &ip)
{
  HTTPClient http;
  http.setTimeout(8000);
  if (!http.begin("http://" + ip + "/api")) return "can't reach " + ip;
  http.addHeader("Content-Type", "application/json");
  int code = http.POST("{\"devicetype\":\"recirc#esp32\"}");
  String body = http.getString();
  http.end();
  int at = body.indexOf("\"username\":\"");
  if (code != 200 || at < 0) {
    if (body.indexOf("link button") >= 0) return "press the round link button on the bridge, then within 30 seconds run \"hue pair " + ip + "\" again";
    return "pairing failed (" + String(code) + ")";
  }
  at += 12;
  cfg.hue_ip = ip;
  cfg.hue_user = body.substring(at, body.indexOf('"', at));
  save_settings();
  hue_have_fingerprint = false;
  hue_error_reported = false;
  return "paired with the Hue bridge at " + ip;
}

void presence_task(void *arg)
{
  unsigned long last_sweep = 0, last_hue = 0;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (millis() - last_hue > 60000 || last_hue == 0) { last_hue = millis(); hue_poll(); }
    if (millis() - last_sweep > 120000 || last_sweep == 0) { last_sweep = millis(); phone_sweep(); }
  }
}

// ---------------------------------------------------------------------------------------------------------------
// The pump

enum Trigger { T_SCHEDULE, T_BUTTON, T_COMMAND };
const char *trigger_names[] = {"schedule", "button", "command"};

bool pump_running = false;
Trigger run_trigger;
unsigned long run_start_ms;
float run_start_F, run_peak_F;
int warm_count;                   // Consecutive readings at or above the stop temperature
unsigned long last_reading_used;
int runs_today = 0;
unsigned long run_seconds_today = 0;
int today = -1;
String last_run_text = "none yet";

void relay(bool on) { digitalWrite(RELAY_PIN, on ? HIGH : LOW); }

void start_run(int why)     // why: T_SCHEDULE, T_BUTTON or T_COMMAND
{
  if (pump_running) return;
  if (!probe_ok()) {
    // A button or command run always alerts; a scheduled one only when "no probe" hasn't just been alerted
    bool quiet = why == T_SCHEDULE && probe_alert_ms != 0 && millis() - probe_alert_ms < 3600000UL;
    log_event(String(quiet ? "" : "alert ") + "recirc " + trigger_names[why] + " run skipped: no return-line temperature");
    return;
  }
  pump_running = true;
  run_trigger = (Trigger)why;
  run_start_ms = millis();
  run_start_F = run_peak_F = return_F;
  warm_count = 0;
  last_reading_used = return_ms;
  relay(true);
}

void stop_run(const char *reason)
{
  if (!pump_running) return;
  relay(false);
  pump_running = false;
  unsigned long secs = (millis() - run_start_ms + 500) / 1000;
  runs_today++;
  run_seconds_today += secs;
  char b[160];
  snprintf(b, sizeof(b), "run %s %s secs %lu start %.1f end %.1f peak %.1f", trigger_names[run_trigger], reason, secs,
           run_start_F, return_F, run_peak_F);
  last_run_text = timestamp() + " " + b;
  out.println(last_run_text);                 // A data line: "YYYY-MM-DD HH:MM:SS run schedule warm secs 34 ..."
}

void watch_run(void)              // Called every 100 ms
{
  if (!pump_running) return;
  unsigned long elapsed = millis() - run_start_ms;
  if (!probe_ok()) { stop_run("probe-failed"); log_event("alert return-line probe stopped answering during a run"); return; }
  if (return_ms != last_reading_used) {       // A new reading
    last_reading_used = return_ms;
    if (return_F > run_peak_F) run_peak_F = return_F;
    warm_count = return_F >= run_start_F + cfg.rise_F ? warm_count + 1 : 0;
  }
  if (warm_count >= 2 && elapsed > 3000) stop_run("warm");
  else if (elapsed >= (unsigned long)cfg.max_run_s * 1000UL) stop_run("max-time");
}

bool quiet_hours(int hour)
{
  if (cfg.quiet_start_h == cfg.quiet_end_h) return false;
  if (cfg.quiet_start_h > cfg.quiet_end_h) return hour >= cfg.quiet_start_h || hour < cfg.quiet_end_h;  // Spans midnight
  return hour >= cfg.quiet_start_h && hour < cfg.quiet_end_h;
}

void schedule(void)               // Called every second
{
  static long last_slot = -1;
  static unsigned long last_untimed_run;
  if (!time_valid) {              // No clock yet: keep the interval from boot, ignore quiet hours
    if (millis() - last_untimed_run >= (unsigned long)cfg.interval_min * 60000UL) {
      last_untimed_run = millis();
      if (someone_home()) start_run(T_SCHEDULE);
    }
    return;
  }
  time_t t = time(nullptr);
  struct tm tm;
  localtime_r(&t, &tm);
  if (tm.tm_yday != today) { today = tm.tm_yday; runs_today = 0; run_seconds_today = 0; }
  long slot = ((long)tm.tm_yday * 24 * 60 + tm.tm_hour * 60 + tm.tm_min) / cfg.interval_min;
  if (slot == last_slot) return;
  bool first = last_slot < 0;
  last_slot = slot;
  if (first) return;              // Don't run just because we booted
  if (quiet_hours(tm.tm_hour)) return;
  if (!someone_home()) return;
  start_run(T_SCHEDULE);
}

void watch_button(void)           // Called every 100 ms; a press of 0.2 s or more starts a run
{
  static int low_count;
  if (digitalRead(BUTTON_PIN) == LOW) {
    if (++low_count == 2) {
      if (pump_running) stop_run("button");
      else start_run(T_BUTTON);
    }
  } else {
    low_count = 0;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Periodic status line, every 5 minutes:
//   YYYY-MM-DD HH:MM:SS status T 64.2 home 1 phone 12 hue 45 lights 3/41 runs 14 secs 410 rssi -62
// phone and hue are minutes since last seen (-1 never); T is "-" without a probe reading.

String minutes_since(unsigned long ms) { return ms == 0 ? String("-1") : String((millis() - ms) / 60000UL); }

void status_line(void)
{
  if (!time_valid) return;
  char t[12];
  if (probe_ok()) snprintf(t, sizeof(t), "%.1f", return_F); else strcpy(t, "-");
  out.println(timestamp() + " status T " + t + " home " + (someone_home() ? "1" : "0") +
              " phone " + minutes_since(last_phone_ms()) + " hue " + minutes_since(hue_change_ms) +
              " lights " + String(hue_lights_on) + "/" + String(hue_lights) +
              " runs " + String(runs_today) + " secs " + String(run_seconds_today) + " rssi " + String(WiFi.RSSI()));
}

// ---------------------------------------------------------------------------------------------------------------
// Commands

String settings_text(void)
{
  char b[200];
  snprintf(b, sizeof(b), "interval %d min, maxrun %d s, rise %.1f F, hold %d min, quiet %d-%d, mode %s, upload %s",
           cfg.interval_min, cfg.max_run_s, cfg.rise_F, cfg.hold_min, cfg.quiet_start_h, cfg.quiet_end_h,
           cfg.mode == 'h' ? "home" : cfg.mode == 'w' ? "away" : "auto", cfg.upload ? "on" : "off");
  return String(b);
}

volatile bool update_check_requested = false;

String run_command(String c)
{
  c.trim();
  int sp = c.indexOf(' ');
  String verb = sp < 0 ? c : c.substring(0, sp);
  String rest = sp < 0 ? "" : c.substring(sp + 1);
  rest.trim();
  if (verb == "run") { start_run(T_COMMAND); return pump_running ? "running" : "not started (no temperature?)"; }
  if (verb == "stop") { stop_run("command"); return "stopped"; }
  if (verb == "mode") {
    if (rest == "auto") cfg.mode = 'a';
    else if (rest == "home") cfg.mode = 'h';
    else if (rest == "away") cfg.mode = 'w';
    else return "mode auto|home|away";
    save_settings();
    log_event("mode " + rest);
    return settings_text();
  }
  if (verb == "set") {
    int sp2 = rest.indexOf(' ');
    if (sp2 < 0) return settings_text();
    String what = rest.substring(0, sp2), v = rest.substring(sp2 + 1);
    if (what == "interval" && v.toInt() >= 5 && v.toInt() <= 240) cfg.interval_min = v.toInt();
    else if (what == "maxrun" && v.toInt() >= 10 && v.toInt() <= 600) cfg.max_run_s = v.toInt();
    else if (what == "rise" && v.toFloat() >= 1 && v.toFloat() <= 30) cfg.rise_F = v.toFloat();
    else if (what == "hold" && v.toInt() >= 5 && v.toInt() <= 24 * 60) cfg.hold_min = v.toInt();
    else if (what == "quiet" && v.indexOf('-') > 0) {
      int a = v.substring(0, v.indexOf('-')).toInt(), b = v.substring(v.indexOf('-') + 1).toInt();
      if (a < 0 || a > 23 || b < 0 || b > 23) return "quiet takes hours 0-23, like 23-7";
      cfg.quiet_start_h = a; cfg.quiet_end_h = b;
    } else return "set interval 5-240 | maxrun 10-600 | rise 1-30 | hold 5-1440 | quiet H-H";
    save_settings();
    log_event("set " + rest);
    return settings_text();
  }
  if (verb == "phone" || verb == "phones") {
    String sub = rest.substring(0, rest.indexOf(' ') < 0 ? rest.length() : rest.indexOf(' '));
    String args = rest.indexOf(' ') < 0 ? "" : rest.substring(rest.indexOf(' ') + 1);
    uint8_t m[6];
    if (sub == "add" || sub == "del") {
      String macs = args.substring(0, args.indexOf(' ') < 0 ? args.length() : args.indexOf(' '));
      String name = args.indexOf(' ') < 0 ? "" : args.substring(args.indexOf(' ') + 1);
      if (!parse_mac(macs, m)) return "phone add <aa:bb:cc:dd:ee:ff> <name> | phone del <mac>";
      String key = mac_text(m), kept;
      int start = 0;
      while (start < (int)cfg.phones.length()) {          // Rebuild the list without this MAC
        int end = cfg.phones.indexOf(';', start);
        if (end < 0) end = cfg.phones.length();
        String item = cfg.phones.substring(start, end);
        if (item.length() && !item.startsWith(key)) kept += (kept.length() ? ";" : "") + item;
        start = end + 1;
      }
      if (sub == "add") {
        name.replace(";", " "); name.replace("=", " ");
        kept += (kept.length() ? ";" : "") + key + "=" + (name.length() ? name : String("phone"));
      }
      cfg.phones = kept;
      save_settings();
      load_phones();
      log_event("phones now " + String(phone_count));
    }
    String r;
    xSemaphoreTake(presence_mutex, portMAX_DELAY);
    for (int i = 0; i < phone_count; i++) {
      r += mac_text(phones[i].mac) + " " + phones[i].name + ", last seen " +
           (phones[i].seen_ms ? String((millis() - phones[i].seen_ms) / 60000UL) + " min ago at " +
                                IPAddress(phones[i].ip).toString() : String("never")) + "\n";
    }
    xSemaphoreGive(presence_mutex);
    return r.length() ? r : String("no phones listed");
  }
  if (verb == "hue") {
    if (rest.startsWith("pair ")) return hue_pair(rest.substring(5));
    if (rest == "forget") { cfg.hue_ip = ""; cfg.hue_user = ""; save_settings(); return "Hue forgotten"; }
    return cfg.hue_user.length() ? "paired with " + cfg.hue_ip + ", " + String(hue_lights_on) + " of " + String(hue_lights) + " lights on"
                                 : String("not paired; hue pair <bridge ip>");
  }
  if (verb == "upload") {
    if (rest == "on" || rest == "off") { cfg.upload = rest == "on"; save_settings(); }
    return String("upload ") + (cfg.upload ? "on" : "off");
  }
  if (verb == "update") { update_check_requested = true; return "checking for new firmware"; }
  if (verb == "reboot") { stop_run("reboot"); log_event("rebooting on command"); delay(300); ESP.restart(); return ""; }
  return "run | stop | mode auto|home|away | set ... | phone add|del ... | phones | hue pair <ip>|forget | upload on|off | update check | reboot";
}

void watch_serial(void)
{
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length()) { out.println(run_command(line)); line = ""; }
    } else if (line.length() < 200) {
      line += c;
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Network: Wi-Fi, the LAN web server, uploads, and pulled firmware updates

WebServer server(80);
bool net_proven = false;          // Reached bedichek.org since boot (decides whether new firmware is kept)
#define GOOD_AFTER_MS (120000UL)  // New firmware must reach bedichek.org within this long of booting, or it rolls back
char reset_cause = 'X';

static bool authorized(void)
{
  if (server.header("X-Token") == RC_LAN_TOKEN) return true;
  server.send(403, "text/plain", "forbidden\n");
  return false;
}

void handle_status(void)
{
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t state = ESP_OTA_IMG_VALID;
  esp_ota_get_state_partition(running, &state);
  String s = "{\n";
  s += "  \"build\": \"" RC_BUILD_ID "\",\n";
  s += "  \"firmware_verified\": " + String(state == ESP_OTA_IMG_PENDING_VERIFY ? "false" : "true") + ",\n";
  s += "  \"time\": \"" + (time_valid ? timestamp() : String("unknown")) + "\",\n";
  s += "  \"uptime_s\": " + String(millis() / 1000) + ",\n";
  s += "  \"reset_cause\": \"" + String(reset_cause) + "\",\n";
  s += "  \"pump\": " + String(pump_running ? "true" : "false") + ",\n";
  s += "  \"return_F\": " + (probe_ok() ? String(return_F, 1) : String("null")) + ",\n";
  s += "  \"probe\": \"" + (probe_found ? rom_text(probe_addr) : String("none")) + "\",\n";
  s += "  \"probe_errors\": " + String(probe_errors) + ",\n";
  s += "  \"someone_home\": " + String(someone_home() ? "true" : "false") + ",\n";
  s += "  \"phone_min_ago\": " + minutes_since(last_phone_ms()) + ",\n";
  s += "  \"hue_change_min_ago\": " + minutes_since(hue_change_ms) + ",\n";
  s += "  \"hue_lights_on\": " + String(hue_lights_on) + ",\n";
  s += "  \"runs_today\": " + String(runs_today) + ",\n";
  s += "  \"run_seconds_today\": " + String(run_seconds_today) + ",\n";
  s += "  \"last_run\": \"" + last_run_text + "\",\n";
  s += "  \"settings\": \"" + settings_text() + "\",\n";
  s += "  \"rssi\": " + String(WiFi.RSSI()) + ",\n";
  s += "  \"ip\": \"" + WiFi.localIP().toString() + "\",\n";
  s += "  \"upload_queue_bytes\": " + String(upload_queue.length()) + ",\n";
  s += "  \"free_heap\": " + String(ESP.getFreeHeap()) + "\n}\n";
  server.send(200, "application/json", s);
}

void handle_log(void)
{
  String s;
  xSemaphoreTake(lines_mutex, portMAX_DELAY);
  recent_lines.copy_tail(s, server.hasArg("bytes") ? server.arg("bytes").toInt() : 8192);
  xSemaphoreGive(lines_mutex);
  server.send(200, "text/plain", s);
}

void handle_cmd(void)
{
  if (!authorized()) return;
  String r = run_command(server.arg("plain"));
  server.send(200, "text/plain", r + "\n");
}

bool update_authorized = false;

void handle_update_upload(void)
{
  HTTPUpload &upload = server.upload();
  // The whole upload is received inside server.handleClient(), so loop() can't feed the 8 s watchdog meanwhile;
  // over a slow link that reset the board mid-upload (2026-11-13).  Feed it per chunk.
  esp_task_wdt_reset();
  if (upload.status == UPLOAD_FILE_START) {
    update_authorized = server.header("X-Token") == RC_LAN_TOKEN;
    if (update_authorized) { stop_run("update"); Update.begin(UPDATE_SIZE_UNKNOWN); }
  } else if (!update_authorized) {
    return;
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    Update.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    Update.end(true);
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
  }
}

void handle_update_done(void)
{
  if (!update_authorized) { server.send(403, "text/plain", "forbidden\n"); return; }
  if (Update.hasError()) { server.send(500, "text/plain", String("update failed: ") + Update.errorString() + "\n"); return; }
  server.send(200, "text/plain", "ok, restarting\n");
  log_event("new firmware loaded from the LAN; restarting");
  delay(500);
  ESP.restart();
}

// Up to two networks are compiled in (rcctl wifi): normally the cabin's, plus a second for bench testing
// elsewhere.  The board joins whichever one it can see, preferring the first, and looks again when Wi-Fi has
// been down for 3 minutes.
struct WifiNet { const char *ssid, *pass; };
const WifiNet WIFI_NETS[] = {
  {RC_WIFI_SSID, RC_WIFI_PASS},
#ifdef RC_WIFI2_SSID
  {RC_WIFI2_SSID, RC_WIFI2_PASS},
#endif
};
const int WIFI_NET_COUNT = sizeof(WIFI_NETS) / sizeof(WIFI_NETS[0]);

void wifi_join(void)
{
  const WifiNet *pick = &WIFI_NETS[0];
  if (WIFI_NET_COUNT > 1) {
    esp_task_wdt_reset();
    int n = WiFi.scanNetworks();           // blocking, about 2 s
    esp_task_wdt_reset();
    bool found = false;
    for (int k = 0; k < WIFI_NET_COUNT && !found; k++)
      for (int i = 0; i < n; i++)
        if (WiFi.SSID(i) == WIFI_NETS[k].ssid) { pick = &WIFI_NETS[k]; found = true; break; }
    WiFi.scanDelete();
  }
  Serial.printf("# joining Wi-Fi %s\n", pick->ssid);
  WiFi.begin(pick->ssid, pick->pass);
}

void watch_wifi(void)
{
  static unsigned long down_since;
  if (WiFi.status() == WL_CONNECTED) { down_since = 0; return; }
  unsigned long now = millis();
  if (down_since == 0) { down_since = now; return; }
  if (WIFI_NET_COUNT > 1 && now - down_since > 180000UL) {
    down_since = now;
    WiFi.disconnect();
    wifi_join();
  }
}

void setup_network(void)
{
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.onEvent([](arduino_event_id_t e, arduino_event_info_t info) {
    if (e == ARDUINO_EVENT_WIFI_STA_GOT_IP) log_event("Wi-Fi up, " + WiFi.localIP().toString() + ", rssi " + String(WiFi.RSSI()));
    if (e == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) Serial.printf("# Wi-Fi lost (reason %d)\n", info.wifi_sta_disconnected.reason);
  });
  wifi_join();
  const char *headers[] = {"X-Token"};
  server.collectHeaders(headers, 1);
  server.on("/", HTTP_GET, handle_status);
  server.on("/status", HTTP_GET, handle_status);
  server.on("/log", HTTP_GET, handle_log);
  server.on("/cmd", HTTP_POST, handle_cmd);
  server.on("/update", HTTP_POST, handle_update_done, handle_update_upload);
  server.begin();
  MDNS.begin(HOSTNAME);
  MDNS.addService("http", "tcp", 80);
}

void upload_task(void *arg)
{
  static char chunk[16384];
  unsigned long backoff_ms = 10000;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(backoff_ms));
    if (WiFi.status() != WL_CONNECTED) continue;
    xSemaphoreTake(lines_mutex, portMAX_DELAY);
    size_t n = upload_queue.peek(chunk, sizeof(chunk));
    xSemaphoreGive(lines_mutex);
    if (n == 0) continue;
    NetworkClientSecure client;
    client.useBuiltinCACertBundle();
    HTTPClient http;
    http.setTimeout(15000);
    int code = -1;
    if (http.begin(client, INGEST_URL)) {
      http.addHeader("Authorization", "Bearer " RC_INGEST_TOKEN);
      http.addHeader("Content-Type", "text/plain");
      code = http.POST((uint8_t *)chunk, n);
      http.end();
    }
    if (code == 200) {
      net_proven = true;
      xSemaphoreTake(lines_mutex, portMAX_DELAY);
      upload_queue.consume(n);
      xSemaphoreGive(lines_mutex);
      backoff_ms = 10000;
    } else {
      Serial.printf("# upload to bedichek.org failed: %d\n", code);
      backoff_ms = backoff_ms < 300000 ? backoff_ms * 2 : 300000;
    }
  }
}

// Pulled updates.  recirc.json is {"build": "...", "size": N, "sha256": "<64 hex>"}, written by "rcctl publish".
static String json_field(const String &j, const char *key)
{
  String k = String("\"") + key + "\"";
  int at = j.indexOf(k);
  if (at < 0) return "";
  at = j.indexOf(':', at + k.length());
  if (at < 0) return "";
  at++;
  while (at < (int)j.length() && (j[at] == ' ' || j[at] == '"')) at++;
  int end = at;
  while (end < (int)j.length() && j[end] != '"' && j[end] != ',' && j[end] != '}' && j[end] != '\n') end++;
  return j.substring(at, end);
}

void check_for_update(void)
{
  if (WiFi.status() != WL_CONNECTED) return;
  NetworkClientSecure client;
  client.useBuiltinCACertBundle();
  HTTPClient http;
  http.setTimeout(15000);
  if (!http.begin(client, FW_BASE "recirc.json")) return;
  http.addHeader("Authorization", "Bearer " RC_FW_TOKEN);
  int code = http.GET();
  String manifest = code == 200 ? http.getString() : "";
  http.end();
  if (code != 200) { Serial.printf("# firmware check failed: %d\n", code); return; }
  net_proven = true;
  String build = json_field(manifest, "build"), want_sha = json_field(manifest, "sha256");
  int size = json_field(manifest, "size").toInt();
  if (build.length() == 0 || build == RC_BUILD_ID || want_sha.length() != 64 || size <= 0) return;
  while (pump_running) vTaskDelay(pdMS_TO_TICKS(1000));   // Never update mid-run
  log_event("downloading firmware " + build);
  if (!http.begin(client, FW_BASE "recirc.bin")) return;
  http.addHeader("Authorization", "Bearer " RC_FW_TOKEN);
  code = http.GET();
  if (code != 200 || http.getSize() != size || !Update.begin(size)) {
    log_event("alert firmware download failed: " + String(code));
    http.end();
    return;
  }
  mbedtls_sha256_context sha;
  mbedtls_sha256_init(&sha);
  mbedtls_sha256_starts(&sha, 0);
  NetworkClient *s = http.getStreamPtr();
  static uint8_t buf[4096];
  int got = 0;
  unsigned long deadline = millis() + 120000;
  while (got < size && millis() < deadline) {
    int n = s->read(buf, min((int)sizeof(buf), size - got));
    if (n <= 0) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
    mbedtls_sha256_update(&sha, buf, n);
    if (Update.write(buf, n) != (size_t)n) break;
    got += n;
  }
  http.end();
  uint8_t digest[32];
  mbedtls_sha256_finish(&sha, digest);
  mbedtls_sha256_free(&sha);
  char hex[65];
  for (int i = 0; i < 32; i++) sprintf(hex + 2 * i, "%02x", digest[i]);
  if (got != size || want_sha != hex) {
    Update.abort();
    log_event("alert firmware " + build + " failed its check (" + String(got) + " of " + String(size) + " bytes); keeping this one");
    return;
  }
  if (!Update.end(true)) { log_event(String("alert firmware install failed: ") + Update.errorString()); return; }
  log_event("installed firmware " + build + "; restarting");
  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP.restart();
}

void update_task(void *arg)
{
  vTaskDelay(pdMS_TO_TICKS(15000));
  unsigned long last = 0;
  for (;;) {
    // Hourly, on request, and every 15 s until bedichek.org has answered once (that is what keeps new firmware)
    bool proving = !net_proven && millis() < GOOD_AFTER_MS;
    if (update_check_requested || last == 0 || millis() - last > (proving ? 15000UL : 3600000UL)) {
      update_check_requested = false;
      last = millis() | 1;
      check_for_update();
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

extern "C" bool verifyRollbackLater() { return true; }  // Keep new images pending until mark_firmware_good()

void mark_firmware_good(void)
{
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t state;
  if (esp_ota_get_state_partition(running, &state) != ESP_OK || state != ESP_OTA_IMG_PENDING_VERIFY) return;
  if (net_proven) {
    esp_ota_mark_app_valid_cancel_rollback();
    log_event("new firmware marked good");
  } else {
    log_event("alert new firmware could not reach bedichek.org; going back to the previous firmware");
    delay(500);
    esp_ota_mark_app_invalid_rollback_and_reboot();
  }
}

void setup_reset_cause(void)
{
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  reset_cause = 'P'; break;
    case ESP_RST_BROWNOUT: reset_cause = 'B'; break;
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      reset_cause = 'W'; break;
    case ESP_RST_SW:       reset_cause = 'S'; break;
    case ESP_RST_PANIC:    reset_cause = 'C'; break;
    default:               reset_cause = 'X'; break;
  }
}

// ---------------------------------------------------------------------------------------------------------------

void setup(void)
{
  relay(false);
  pinMode(RELAY_PIN, OUTPUT);
  relay(false);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(SERIAL_BAUD);
  lines_mutex = xSemaphoreCreateMutex();
  presence_mutex = xSemaphoreCreateMutex();
  recent_lines.init(32 * 1024);
  upload_queue.init(512 * 1024);            // In PSRAM: days of telemetry if bedichek.org is unreachable
  load_settings();
  load_phones();
  setup_reset_cause();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  Wire.setTimeOut(25);
  setup_time();
  log_event(String(reset_cause == 'W' || reset_cause == 'C' || reset_cause == 'B' ? "alert " : "") +
            "reboot recirc " RC_BUILD_ID " reset cause " + reset_cause);
  if (!time_valid) log_event("RTC has no valid time; waiting for NTP");
  find_probe();
  setup_network();
  xTaskCreatePinnedToCore(upload_task, "upload", 8192, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(update_task, "update", 8192, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(presence_task, "presence", 8192, nullptr, 1, nullptr, 0);
  esp_task_wdt_config_t wdt_config = {.timeout_ms = 8000, .idle_core_mask = 0, .trigger_panic = true};
  esp_task_wdt_reconfigure(&wdt_config);
  esp_task_wdt_add(nullptr);
}

void loop(void)
{
  static unsigned long last_100ms, last_1s, last_status;
  static bool good_checked = false;
  esp_task_wdt_reset();
  server.handleClient();
  unsigned long now = millis();
  if (now - last_100ms >= 100) {
    last_100ms = now;
    read_probe();
    watch_run();
    watch_button();
    watch_serial();
  }
  if (now - last_1s >= 1000) {
    last_1s = now;
    if (ntp_synced) {
      ntp_synced = false;
      if (!time_valid) { time_valid = true; log_event("clock set from NTP"); }
      static unsigned long last_rtc_write;
      if (last_rtc_write == 0 || now - last_rtc_write > 3600000UL) { if (rtc_write(time(nullptr))) last_rtc_write = now; }
    }
    schedule();
    watch_wifi();
    if (!good_checked && (net_proven || now > GOOD_AFTER_MS)) { good_checked = true; mark_firmware_good(); }
  }
  if (now - last_status >= 300000 || last_status == 0) {
    last_status = now;
    status_line();
  }
  delay(2);
}
