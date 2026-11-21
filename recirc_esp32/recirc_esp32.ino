// recirc_esp32: hot-water recirculation pump controller (Rob Bedichek, started 2026-11-10).
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
//   Someone is home when any of these holds:
//   - a family phone has been seen on the Wi-Fi within hold_min (60) minutes (ARP sweep of the local /24 for their MACs);
//   - a Hue light was switched or dimmed within hue_hold_min (120) minutes (polled from the bridges' local API);
//   - a guest device arrived within guest_hold_min (120) minutes ("guests", on by default): any other device with a
//     private (randomized) Wi-Fi address, as visitors' phones use, that had been gone for hold_min minutes or more.
//     Arrival, not presence, because a house collects devices that never leave (an iPad, a TV); fixed gear uses
//     real MACs and never counts.  Devices seen in the first 10 minutes after boot were already there.
//   With all of these off it assumes someone is home.  "mode home|away" overrides.
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
//   set interval|maxrun|rise|hold|huehold|guesthold|quiet <value>   (quiet takes "23-7"; "set" alone lists them)
//   phone add <mac> <name> | phone del <mac> | phones
//   guests [on|off] | guest ignore|unignore <mac>   (a private-address device that is always there, e.g. an iPad)
//   hue pair <ip> (press the bridge's link button first; up to 4 bridges) | hue forget [ip] | hue (list bridges)
//   hue batteries                   battery level of each Hue dimmer, button and motion sensor, with names
//   hue lights                      every bulb: name, maker, model, firmware, reachable
//   hue delete-light <bridge>.<id>  remove a bulb the bridge reports unreachable (a ghost of a replaced bulb)
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
#include "hue_json.h"  // Streaming scanner for the Hue bridges' replies

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
  int hold_min = 60;          // Someone is home for this long after a family phone was last seen; also how long a
                              // guest device must be gone before its return counts as an arrival
  int hue_hold_min = 120;     // ... for this long after a Hue light was switched or dimmed
  int guest_hold_min = 120;   // ... for this long after a guest device arrived
  int quiet_start_h = 23;     // No scheduled runs from quiet_start_h:00 ...
  int quiet_end_h = 7;        // ... to quiet_end_h:00, local time
  char mode = 'a';            // 'a' auto (presence), 'h' always home, 'w' away (no scheduled runs)
  bool upload = false;        // Send telemetry to bedichek.org
  String phones;              // "aa:bb:cc:dd:ee:ff=name;..." lower case
  bool guests = true;         // Any device with a private (randomized) Wi-Fi address counts too: guests' phones
  String ignore;              // "aa:bb:cc:dd:ee:ff;..." private-address devices that never mean someone is home
  String hue;                 // "ip key\n" per paired Hue bridge (key from "hue pair")
} cfg;

Preferences prefs;

void save_settings(void)
{
  prefs.begin("recirc", false);
  prefs.putInt("interval", cfg.interval_min);
  prefs.putInt("maxrun", cfg.max_run_s);
  prefs.putFloat("rise", cfg.rise_F);
  prefs.putInt("hold", cfg.hold_min);
  prefs.putInt("huehold", cfg.hue_hold_min);
  prefs.putInt("guesthold", cfg.guest_hold_min);
  prefs.putInt("qstart", cfg.quiet_start_h);
  prefs.putInt("qend", cfg.quiet_end_h);
  prefs.putChar("mode", cfg.mode);
  prefs.putBool("upload", cfg.upload);
  prefs.putString("phones", cfg.phones);
  prefs.putBool("guests", cfg.guests);
  prefs.putString("ignore", cfg.ignore);
  prefs.putString("hue", cfg.hue);
  if (prefs.isKey("hue_ip")) prefs.remove("hue_ip");          // Replaced by "hue"
  if (prefs.isKey("hue_user")) prefs.remove("hue_user");
  prefs.end();
}

void load_settings(void)
{
  prefs.begin("recirc", true);
  cfg.interval_min = prefs.getInt("interval", cfg.interval_min);
  cfg.max_run_s = prefs.getInt("maxrun", cfg.max_run_s);
  cfg.rise_F = prefs.getFloat("rise", cfg.rise_F);
  cfg.hold_min = prefs.getInt("hold", cfg.hold_min);
  cfg.hue_hold_min = prefs.getInt("huehold", cfg.hue_hold_min);
  cfg.guest_hold_min = prefs.getInt("guesthold", cfg.guest_hold_min);
  cfg.quiet_start_h = prefs.getInt("qstart", cfg.quiet_start_h);
  cfg.quiet_end_h = prefs.getInt("qend", cfg.quiet_end_h);
  cfg.mode = prefs.getChar("mode", cfg.mode);
  cfg.upload = prefs.getBool("upload", cfg.upload);
  cfg.phones = prefs.getString("phones", "");
  cfg.guests = prefs.getBool("guests", cfg.guests);
  cfg.ignore = prefs.getString("ignore", "");
  cfg.hue = prefs.getString("hue", "");
  if (cfg.hue.length() == 0 && prefs.getString("hue_user", "").length())     // One bridge, from before 2026-11-17
    cfg.hue = prefs.getString("hue_ip", "") + " " + prefs.getString("hue_user", "") + "\n";
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
unsigned long hue_change_ms = 0;      // millis() of the last Hue light change on any bridge, 0 if none since boot
int hue_lights = -1, hue_lights_on = -1;   // Totals over the bridges

// Guests: any other device answering ARP with a locally administered (private, randomized) MAC.  Phones use these
// on Wi-Fi; routers, Hue bridges, cameras and computers left on use their real MACs, so they never count.
struct Guest { uint8_t mac[6]; unsigned long seen_ms; uint32_t ip; };
Guest guests[24];
int guest_count = 0;
uint8_t ignored[16][6];
int ignored_count = 0;
int arrivals_phone = 0, arrivals_guest = 0;  // Counted in lwIP's thread (presence_mutex held), logged by the presence task
unsigned long guest_arrival_ms = 0;   // millis() a guest device last arrived, 0 if none since boot
#define BOOT_GRACE_MS (10 * 60000UL)  // Guest devices first seen this soon after boot were already there

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

void load_ignores(void)           // From cfg.ignore; also drops those devices from the guest table
{
  xSemaphoreTake(presence_mutex, portMAX_DELAY);
  ignored_count = 0;
  int start = 0;
  while (start < (int)cfg.ignore.length() && ignored_count < 16) {
    int end = cfg.ignore.indexOf(';', start);
    if (end < 0) end = cfg.ignore.length();
    if (parse_mac(cfg.ignore.substring(start, end), ignored[ignored_count])) ignored_count++;
    start = end + 1;
  }
  for (int i = 0; i < guest_count; ) {
    bool drop = false;
    for (int k = 0; k < ignored_count; k++) if (memcmp(guests[i].mac, ignored[k], 6) == 0) drop = true;
    if (drop) guests[i] = guests[--guest_count]; else i++;
  }
  xSemaphoreGive(presence_mutex);
}

bool private_mac(const uint8_t m[6]) { return (m[0] & 0x02) && !(m[0] & 0x01); }   // Locally administered unicast

unsigned long last_guest_ms(void)     // millis() a guest device was last seen, 0 if never
{
  unsigned long best = 0;
  xSemaphoreTake(presence_mutex, portMAX_DELAY);
  for (int i = 0; i < guest_count; i++)
    if (best == 0 || (long)(guests[i].seen_ms - best) > 0) best = guests[i].seen_ms;
  xSemaphoreGive(presence_mutex);
  return best;
}

int guests_present(void)             // Guest devices seen within the hold time
{
  int n = 0;
  xSemaphoreTake(presence_mutex, portMAX_DELAY);
  for (int i = 0; i < guest_count; i++)
    if (millis() - guests[i].seen_ms < (unsigned long)cfg.hold_min * 60000UL) n++;
  xSemaphoreGive(presence_mutex);
  return n;
}

unsigned long last_guest_arrival_ms(void)
{
  xSemaphoreTake(presence_mutex, portMAX_DELAY);
  unsigned long t = guest_arrival_ms;
  xSemaphoreGive(presence_mutex);
  return t;
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
  bool have_phones = phone_count > 0, have_hue = cfg.hue.length() > 0;
  if (!have_phones && !have_hue && !cfg.guests) return true;   // Nothing to go on: assume yes
  return recent(last_phone_ms(), cfg.hold_min) || recent(hue_change_ms, cfg.hue_hold_min) ||
         (cfg.guests && recent(last_guest_arrival_ms(), cfg.guest_hold_min));
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
    unsigned long now = millis(), hold = (unsigned long)cfg.hold_min * 60000UL;
    bool family = false;
    for (int p = 0; p < phone_count; p++) {
      if (memcmp(phones[p].mac, eth->addr, 6) == 0) {
        if (phones[p].seen_ms == 0 || now - phones[p].seen_ms >= hold) arrivals_phone++;
        phones[p].seen_ms = now;
        phones[p].ip = ip->addr;
        family = true;
      }
    }
    if (family || !cfg.guests || !private_mac(eth->addr)) continue;
    bool skip = false;
    for (int k = 0; k < ignored_count; k++) if (memcmp(ignored[k], eth->addr, 6) == 0) skip = true;
    if (skip) continue;
    int g = 0;
    while (g < guest_count && memcmp(guests[g].mac, eth->addr, 6) != 0) g++;
    if (g == guest_count) {                       // New: take a free slot, or the one seen longest ago
      if (guest_count < 24) guest_count++;
      else {
        g = 0;
        for (int k = 1; k < 24; k++) if ((long)(guests[k].seen_ms - guests[g].seen_ms) < 0) g = k;
      }
      memcpy(guests[g].mac, eth->addr, 6);
      guests[g].seen_ms = 0;
    }
    if (guests[g].seen_ms == 0 || now - guests[g].seen_ms >= hold) {
      arrivals_guest++;
      if (now >= BOOT_GRACE_MS) guest_arrival_ms = now;
    }
    guests[g].seen_ms = now;
    guests[g].ip = ip->addr;
  }
  xSemaphoreGive(job->done);
}

void phone_sweep(void)            // ARP every address in our /24, a few at a time so the small ARP table keeps up
{
  if ((phone_count == 0 && !cfg.guests) || WiFi.status() != WL_CONNECTED) return;
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

// Hue: up to four bridges.  Every minute, GET /api/<key>/lights from each and fingerprint every
// light's "on" and "bri": a changed fingerprint means someone switched or dimmed a light.  Streams the reply rather
// than parsing it whole, since big houses send 50 KB+.
//
// Once after boot and then daily at 09:00, read each bridge's battery-powered accessories (dimmers, buttons,
// motion sensors) and its Zigbee channel, and log data lines:
//   YYYY-MM-DD HH:MM:SS hue battery 143.2 87 reachable 1 model RWL021 fw 2.45.2_hF4400CA
//   YYYY-MM-DD HH:MM:SS hue radio 143 zigbee 25 wifi 6
//   YYYY-MM-DD HH:MM:SS hue light 143.3 maker Signify_Netherlands_B.V. model LCA001 fw 1.122.2 reachable 1
// (a bulb's line only at the first reading after boot and when its maker, model or firmware changes; spaces in
// values become "_").
// 143.2 is the bridge's last address byte and the accessory's id on it; "hue batteries" shows their names, which stay
// out of recirc.txt because it is public.  wifi is the 2.4 GHz channel of the access point this board is on.
// Every hour it also checks them without logging, and alerts when one that was reachable becomes unreachable: the
// switch has stopped working (usually a cell sagging under load).  A v2 dimmer's % runs from about 3.0 V (0%) to
// 3.25 V (100%): Rob's office dimmer read 1% at 3.05 V and 100% on a fresh cell (2026-11-18).

// struct Hue_Bridge is in hue_json.h, ahead of the prototypes the Arduino build adds.
#define HUE_MAX_BRIDGES (4)
#define HUE_MAX_SENSORS (40)
Hue_Bridge bridges[HUE_MAX_BRIDGES];
int bridge_count = 0;
volatile bool hue_reload = true;       // cfg.hue changed: rebuild bridges[] (presence task)
volatile bool hue_batt_pending = true; // Read the batteries soon (after boot or pairing)

struct Hue_Battery { Hue_Sensor s; int bridge_octet; };
Hue_Battery hue_batts[HUE_MAX_SENSORS];
int hue_batt_count = 0;

// Bulbs, read with the batteries.  A "hue light" line goes out for each one at the first reading after boot and
// whenever its maker, model or firmware changes, so a firmware update shows without 100+ lines a day.
#define HUE_MAX_LIGHTS (200)
struct Hue_Light_Entry { Hue_Light l; int bridge_octet; uint32_t logged; };
Hue_Light_Entry hue_lights_tab[HUE_MAX_LIGHTS];
int hue_light_count = 0;
SemaphoreHandle_t hue_mutex;           // Guards bridges[] counts and hue_batts[] for the command handlers

int ip_octet(const String &ip) { int dot = ip.lastIndexOf('.'); return dot < 0 ? 0 : ip.substring(dot + 1).toInt(); }

void load_bridges(void)                // From cfg.hue; presence task only
{
  xSemaphoreTake(hue_mutex, portMAX_DELAY);
  bridge_count = 0;
  int at = 0;
  while (at < (int)cfg.hue.length() && bridge_count < HUE_MAX_BRIDGES) {
    int nl = cfg.hue.indexOf('\n', at);
    if (nl < 0) nl = cfg.hue.length();
    String line = cfg.hue.substring(at, nl);
    at = nl + 1;
    int sp = line.indexOf(' ');
    if (sp <= 0) continue;
    Hue_Bridge &b = bridges[bridge_count++];
    b.ip = line.substring(0, sp);
    b.key = line.substring(sp + 1);
    b.have_fingerprint = b.error_reported = false;
    b.lights = b.lights_on = b.zigbee_channel = -1;
  }
  xSemaphoreGive(hue_mutex);
}

// GET http://<bridge>/api/<key>/<what> and feed the reply to j.  False (after one alert per outage) on failure.
bool hue_get(Hue_Bridge &b, const char *what, Json_Scan &j, bool *saw_error)
{
  HTTPClient http;
  http.setTimeout(8000);
  if (!http.begin("http://" + b.ip + "/api/" + b.key + "/" + what)) return false;
  int code = http.GET();
  if (code != 200) {
    if (!b.error_reported) { log_event("alert Hue bridge at " + b.ip + " answered " + String(code)); b.error_reported = true; }
    http.end();
    return false;
  }
  NetworkClient *s = http.getStreamPtr();
  char win[7] = {0};                          // The last six characters seen, to spot {"error": ...}
  unsigned long deadline = millis() + 8000;
  int remaining = http.getSize();             // -1 when chunked
  while (http.connected() && millis() < deadline && remaining != 0) {
    int c = s->read();
    if (c < 0) { delay(2); continue; }
    if (remaining > 0) remaining--;
    memmove(win, win + 1, 5);
    win[5] = (char)c;
    if (memcmp(win, "error\"", 6) == 0) *saw_error = true;
    j.feed((char)c);
  }
  http.end();
  return true;
}

// Fingerprint of the lights: FNV-1a over every "on" value, and "bri" of lights that are on (a light that is off can
// report a new brightness with nobody touching it).  The bridge sends "on" before "bri" in each state.
struct Light_Scan {
  uint32_t h = 2166136261u;
  int lights = 0, on = 0;
  bool last_on = false;
  static void value(void *ctx, int depth, const char *key, const char *v)
  {
    Light_Scan *l = (Light_Scan *)ctx;
    if (depth != 3) return;                   // {"1": {"state": {"on": true, "bri": 254, ...}, ...}, ...}
    bool is_on = !strcmp(key, "on");
    if (!is_on && strcmp(key, "bri")) return;
    if (is_on) { l->lights++; l->last_on = !strcmp(v, "true"); if (l->last_on) l->on++; }
    else if (!l->last_on) return;
    for (const char *p = v; *p; p++) l->h = (l->h ^ (uint8_t)*p) * 16777619u;
    l->h = (l->h ^ '|') * 16777619u;
  }
};

void hue_poll(void)
{
  if (WiFi.status() != WL_CONNECTED) return;
  int total = 0, total_on = 0, answered = 0;
  for (int i = 0; i < bridge_count; i++) {
    Hue_Bridge &b = bridges[i];
    Json_Scan j;
    Light_Scan l;
    j.ctx = &l; j.on_value = Light_Scan::value;
    bool saw_error = false;
    if (!hue_get(b, "lights", j, &saw_error)) continue;
    if (saw_error && l.lights == 0) {
      if (!b.error_reported) { log_event("alert the Hue bridge at " + b.ip + " rejected our key; run \"hue pair " + b.ip + "\" again"); b.error_reported = true; }
      continue;
    }
    b.error_reported = false;
    // A different light count is a short read or a light added or removed, not someone at a switch.
    if (b.have_fingerprint && l.lights == b.lights && l.h != b.fingerprint) {
      hue_change_ms = millis();
      log_event("hue change at " + b.ip + ": " + String(l.on) + " of " + String(l.lights) + " lights on");
    }
    b.fingerprint = l.h;
    b.have_fingerprint = true;
    b.lights = l.lights;
    b.lights_on = l.on;
    total += l.lights;
    total_on += l.on;
    answered++;
  }
  hue_lights = answered ? total : -1;
  hue_lights_on = answered ? total_on : -1;
}

static String no_spaces(const char *v) { String t = v[0] ? v : "-"; t.replace(' ', '_'); return t; }

static uint32_t fnv(const char *a, uint32_t h = 2166136261u)
{
  for (; *a; a++) h = (h ^ (uint8_t)*a) * 16777619u;
  return (h ^ '|') * 16777619u;
}

void hue_lights_read(Hue_Bridge &b)  // One bridge's bulbs into hue_lights_tab[]; logs new or changed ones
{
  static Hue_Light found[HUE_MAX_LIGHTS];
  Json_Scan j;
  Hue_Light_Scan ls;
  bool saw_error = false;
  ls.attach(j, found, HUE_MAX_LIGHTS);
  if (!hue_get(b, "lights", j, &saw_error) || ls.count == 0) return;
  int octet = ip_octet(b.ip);
  for (int k = 0; k < ls.count; k++) {
    Hue_Light &l = found[k];
    uint32_t h = fnv(l.fw, fnv(l.model, fnv(l.maker)));
    xSemaphoreTake(hue_mutex, portMAX_DELAY);
    int m = 0;
    while (m < hue_light_count && !(hue_lights_tab[m].bridge_octet == octet && !strcmp(hue_lights_tab[m].l.id, l.id))) m++;
    bool fresh = m == hue_light_count;
    if (fresh && hue_light_count < HUE_MAX_LIGHTS) { hue_light_count++; hue_lights_tab[m].logged = 0; }
    bool log_it = m < HUE_MAX_LIGHTS && (fresh || hue_lights_tab[m].logged != h);
    if (m < HUE_MAX_LIGHTS) { hue_lights_tab[m].l = l; hue_lights_tab[m].bridge_octet = octet; hue_lights_tab[m].logged = h; }
    xSemaphoreGive(hue_mutex);
    if (log_it)
      out.println(timestamp() + " hue light " + String(octet) + "." + l.id + " maker " + no_spaces(l.maker) +
                  " model " + no_spaces(l.model) + " fw " + no_spaces(l.fw) + " reachable " + String(l.reachable));
  }
}

String hue_lights_text(void)         // For the "hue lights" command: includes names
{
  String r;
  xSemaphoreTake(hue_mutex, portMAX_DELAY);
  for (int i = 0; i < hue_light_count; i++) {
    Hue_Light &l = hue_lights_tab[i].l;
    r += String(hue_lights_tab[i].bridge_octet) + "." + l.id + "\t" + l.name + "\t" + l.maker + "\t" + l.model + "\t" +
         l.product + "\t" + l.fw + "\t" + (l.reachable == 0 ? "unreachable" : "ok") + "\n";
  }
  xSemaphoreGive(hue_mutex);
  return r.length() ? r : String("no bulbs read yet\n");
}

void hue_battery_read(bool log_lines)   // log_lines: the daily data lines; false for the hourly reachability check
{
  if (WiFi.status() != WL_CONNECTED || !time_valid) return;
  static Hue_Battery fresh[HUE_MAX_SENSORS];
  int n = 0;
  for (int i = 0; i < bridge_count; i++) {
    Hue_Bridge &b = bridges[i];
    bool saw_error = false;
    Json_Scan jc;
    Hue_Config_Scan c;
    c.attach(jc);
    if (hue_get(b, "config", jc, &saw_error)) b.zigbee_channel = c.zigbee_channel;
    if (log_lines)
      out.println(timestamp() + " hue radio " + String(ip_octet(b.ip)) + " zigbee " + String(b.zigbee_channel) +
                  " wifi " + String(WiFi.channel()));

    static Hue_Sensor found[HUE_MAX_SENSORS];
    Json_Scan js;
    Hue_Sensor_Scan ss;
    ss.attach(js, found, HUE_MAX_SENSORS - n);
    if (!hue_get(b, "sensors", js, &saw_error)) continue;
    for (int k = 0; k < ss.count; k++) {
      Hue_Battery &h = fresh[n++];
      h.s = found[k];
      h.bridge_octet = ip_octet(b.ip);
      int was = -1;                                       // Reachable at the last reading since boot, -1 if unknown
      xSemaphoreTake(hue_mutex, portMAX_DELAY);
      for (int m = 0; m < hue_batt_count; m++)
        if (hue_batts[m].bridge_octet == h.bridge_octet && !strcmp(hue_batts[m].s.id, h.s.id)) was = hue_batts[m].s.reachable;
      xSemaphoreGive(hue_mutex);
      String tag = String(h.bridge_octet) + "." + h.s.id;
      if (log_lines)
        out.println(timestamp() + " hue battery " + tag + " " + String(h.s.battery) + " reachable " + String(h.s.reachable) +
                    " model " + (h.s.model[0] ? h.s.model : "-") + " fw " + (h.s.fw[0] ? h.s.fw : "-"));
      // Only a change seen since boot alerts, so reboots don't repeat alerts for accessories long gone.
      if (was == 1 && h.s.reachable == 0)
        log_event("alert Hue " + String(h.s.model) + " " + tag + " stopped responding (\"hue batteries\" names it)");
      else if (was == 0 && h.s.reachable == 1)
        log_event("Hue " + String(h.s.model) + " " + tag + " is responding again");
    }
    if (log_lines) hue_lights_read(b);
  }
  xSemaphoreTake(hue_mutex, portMAX_DELAY);
  memcpy(hue_batts, fresh, sizeof(Hue_Battery) * n);
  hue_batt_count = n;
  xSemaphoreGive(hue_mutex);
}

String hue_batteries_text(void)      // For the "hue batteries" command (LAN token or USB only): includes names
{
  String r;
  xSemaphoreTake(hue_mutex, portMAX_DELAY);
  for (int i = 0; i < hue_batt_count; i++) {
    Hue_Battery &h = hue_batts[i];
    r += String(h.bridge_octet) + "." + h.s.id + "  " + String(h.s.battery) + "%  " + (h.s.reachable == 0 ? "unreachable  " : "") +
         h.s.model + "  " + h.s.name + (h.s.last[0] ? String("  last event ") + h.s.last + " UTC" : String("")) + "\n";
  }
  xSemaphoreGive(hue_mutex);
  return r.length() ? r : String("no battery readings yet\n");
}

String hue_bridges_text(void)
{
  if (cfg.hue.length() == 0) return "not paired; hue pair <bridge ip>";
  String r;
  xSemaphoreTake(hue_mutex, portMAX_DELAY);
  for (int i = 0; i < bridge_count; i++)
    r += bridges[i].ip + ": " + String(bridges[i].lights_on) + " of " + String(bridges[i].lights) + " lights on, zigbee channel " +
         String(bridges[i].zigbee_channel) + "\n";
  xSemaphoreGive(hue_mutex);
  return r + "this board's Wi-Fi channel " + String(WiFi.channel());
}

String hue_set_bridge(const String &ip, const String &key)    // key "" removes the bridge
{
  String kept, line;
  int at = 0;
  while (at < (int)cfg.hue.length()) {
    int nl = cfg.hue.indexOf('\n', at);
    if (nl < 0) nl = cfg.hue.length();
    line = cfg.hue.substring(at, nl);
    at = nl + 1;
    if (line.length() && !line.startsWith(ip + " ")) kept += line + "\n";
  }
  if (key.length()) kept += ip + " " + key + "\n";
  cfg.hue = kept;
  save_settings();
  hue_reload = true;
  hue_batt_pending = true;
  return kept;
}

// "hue delete-light 143.19": remove a bulb from its bridge, for entries left behind by bulbs that were replaced.
// Refuses unless the bridge says, right now, that the bulb is unreachable: a working bulb is never deleted.
String hue_delete_light(const String &tag)
{
  int dot = tag.indexOf('.');
  if (dot <= 0) return "hue delete-light <bridge>.<id>, e.g. 143.19 (from hue lights)";
  int octet = tag.substring(0, dot).toInt();
  String id = tag.substring(dot + 1);
  for (unsigned i = 0; i < id.length(); i++) if (!isDigit(id[i])) return "bad light id";
  Hue_Bridge *b = nullptr;
  for (int i = 0; i < bridge_count; i++) if (ip_octet(bridges[i].ip) == octet) b = &bridges[i];
  if (b == nullptr) return "no paired bridge ending in ." + String(octet);
  String url = "http://" + b->ip + "/api/" + b->key + "/lights/" + id;
  HTTPClient http;
  http.setTimeout(8000);
  if (!http.begin(url)) return "can't reach the bridge";
  int code = http.GET();
  String body = http.getString();
  http.end();
  if (code != 200 || body.indexOf("\"error\"") >= 0) return "light " + tag + " not found (" + String(code) + ")";
  String compact = body;
  compact.replace(" ", "");
  if (compact.indexOf("\"reachable\":false") < 0) return "light " + tag + " is reachable: not deleting it";
  int at = body.indexOf("\"name\"");
  String name = at < 0 ? String("?") : body.substring(body.indexOf('"', body.indexOf(':', at)) + 1);
  name = name.substring(0, name.indexOf('"'));
  if (!http.begin(url)) return "can't reach the bridge";
  code = http.sendRequest("DELETE");
  body = http.getString();
  http.end();
  if (code != 200 || body.indexOf("success") < 0) return "delete failed (" + String(code) + "): " + body.substring(0, 120);
  xSemaphoreTake(hue_mutex, portMAX_DELAY);           // Drop it from the bulb table too
  for (int m = 0; m < hue_light_count; m++)
    if (hue_lights_tab[m].bridge_octet == octet && id == hue_lights_tab[m].l.id) { hue_lights_tab[m] = hue_lights_tab[--hue_light_count]; break; }
  xSemaphoreGive(hue_mutex);
  log_event("hue light " + tag + " deleted (unreachable)");
  return "deleted light " + tag + " (" + name + ") from the bridge at " + b->ip;
}

String hue_pair(const String &ip)
{
  int lines = 0;
  for (unsigned i = 0; i < cfg.hue.length(); i++) if (cfg.hue[i] == '\n') lines++;
  if (lines >= HUE_MAX_BRIDGES && cfg.hue.indexOf(ip + " ") < 0) return "already paired with " + String(HUE_MAX_BRIDGES) + " bridges";
  HTTPClient http;
  http.setTimeout(8000);
  if (!http.begin("http://" + ip + "/api")) return "can't reach " + ip;
  http.addHeader("Content-Type", "application/json");
  int code = http.POST("{\"devicetype\":\"recirc#esp32\"}");
  String body = http.getString();
  http.end();
  struct Find { char user[48] = ""; } f;      // [{"success": {"username": "<key>"}}]
  Json_Scan j;
  j.ctx = &f;
  j.on_value = [](void *ctx, int, const char *key, const char *v) { if (!strcmp(key, "username")) strncpy(((Find *)ctx)->user, v, 47); };
  for (unsigned i = 0; i < body.length(); i++) j.feed(body[i]);
  if (code != 200 || !f.user[0]) {
    if (body.indexOf("link button") >= 0) return "press the round link button on the bridge, then within 30 seconds run \"hue pair " + ip + "\" again";
    return "pairing failed (" + String(code) + ")";
  }
  hue_set_bridge(ip, f.user);
  return "paired with the Hue bridge at " + ip;
}

void presence_task(void *arg)
{
  unsigned long last_sweep = 0, last_hue = 0;
  int batt_yday = -1;
  unsigned long last_reach = 0;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (hue_reload) { hue_reload = false; load_bridges(); last_hue = 0; }
    if (millis() - last_hue > 60000 || last_hue == 0) { last_hue = millis(); hue_poll(); }
    if (bridge_count && time_valid && millis() > 180000) {   // Batteries: 3 min after boot or pairing, then 09:00 daily
      time_t t = time(nullptr);
      struct tm tm;
      localtime_r(&t, &tm);
      if (hue_batt_pending || (tm.tm_hour >= 9 && tm.tm_yday != batt_yday)) {
        hue_batt_pending = false;
        if (tm.tm_hour >= 9) batt_yday = tm.tm_yday;
        hue_battery_read(true);
        last_reach = millis();
      } else if (millis() - last_reach > 3600000UL) {
        last_reach = millis();
        hue_battery_read(false);
      }
    }
    if (millis() - last_sweep > 120000 || last_sweep == 0) {
      last_sweep = millis();
      phone_sweep();
      // "YYYY-MM-DD HH:MM:SS arrive guest 1 present 2": devices not seen within the hold time came back.  No MACs
      // or names: recirc.txt is public.
      if (time_valid && (arrivals_phone || arrivals_guest)) {
        int p = arrivals_phone, g = arrivals_guest;
        arrivals_phone = arrivals_guest = 0;
        if (p) out.println(timestamp() + " arrive phone " + String(p) + " present " + String(guests_present()));
        if (g) out.println(timestamp() + " arrive guest " + String(g) + " present " + String(guests_present()));
      }
    }
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
//   YYYY-MM-DD HH:MM:SS status T 64.2 home 1 phone 12 guest 3 arrive 40 devices 2 hue 45 lights 3/41 runs 14 secs 410 rssi -62
// phone, guest and hue are minutes since last seen (-1 never); arrive is minutes since a guest device arrived;
// devices is guest devices seen within the hold time;
// T is "-" without a probe reading.

String minutes_since(unsigned long ms) { return ms == 0 ? String("-1") : String((millis() - ms) / 60000UL); }

void status_line(void)
{
  if (!time_valid) return;
  char t[12];
  if (probe_ok()) snprintf(t, sizeof(t), "%.1f", return_F); else strcpy(t, "-");
  out.println(timestamp() + " status T " + t + " home " + (someone_home() ? "1" : "0") +
              " phone " + minutes_since(last_phone_ms()) + " guest " + minutes_since(last_guest_ms()) +
              " arrive " + minutes_since(last_guest_arrival_ms()) +
              " devices " + String(guests_present()) + " hue " + minutes_since(hue_change_ms) +
              " lights " + String(hue_lights_on) + "/" + String(hue_lights) +
              " runs " + String(runs_today) + " secs " + String(run_seconds_today) + " rssi " + String(WiFi.RSSI()));
}

// ---------------------------------------------------------------------------------------------------------------
// Commands

String settings_text(void)
{
  char b[260];
  snprintf(b, sizeof(b), "interval %d min, maxrun %d s, rise %.1f F, hold %d min, huehold %d min, guesthold %d min, "
           "quiet %d-%d, mode %s, guests %s, upload %s",
           cfg.interval_min, cfg.max_run_s, cfg.rise_F, cfg.hold_min, cfg.hue_hold_min, cfg.guest_hold_min,
           cfg.quiet_start_h, cfg.quiet_end_h,
           cfg.mode == 'h' ? "home" : cfg.mode == 'w' ? "away" : "auto", cfg.guests ? "on" : "off", cfg.upload ? "on" : "off");
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
    else if (what == "huehold" && v.toInt() >= 5 && v.toInt() <= 24 * 60) cfg.hue_hold_min = v.toInt();
    else if (what == "guesthold" && v.toInt() >= 5 && v.toInt() <= 24 * 60) cfg.guest_hold_min = v.toInt();
    else if (what == "quiet" && v.indexOf('-') > 0) {
      int a = v.substring(0, v.indexOf('-')).toInt(), b = v.substring(v.indexOf('-') + 1).toInt();
      if (a < 0 || a > 23 || b < 0 || b > 23) return "quiet takes hours 0-23, like 23-7";
      cfg.quiet_start_h = a; cfg.quiet_end_h = b;
    } else return "set interval 5-240 | maxrun 10-600 | rise 1-30 | hold|huehold|guesthold 5-1440 | quiet H-H";
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
  if (verb == "guests" || verb == "guest") {
    if (rest == "on" || rest == "off") { cfg.guests = rest == "on"; save_settings(); log_event("guests " + rest); }
    else if (rest.startsWith("ignore ") || rest.startsWith("unignore ")) {
      bool add = rest.startsWith("ignore ");
      uint8_t m[6];
      if (!parse_mac(rest.substring(rest.indexOf(' ') + 1), m)) return "guest ignore|unignore <aa:bb:cc:dd:ee:ff>";
      String key = mac_text(m), kept;
      int start = 0;
      while (start < (int)cfg.ignore.length()) {
        int end = cfg.ignore.indexOf(';', start);
        if (end < 0) end = cfg.ignore.length();
        String item = cfg.ignore.substring(start, end);
        if (item.length() && item != key) kept += (kept.length() ? ";" : "") + item;
        start = end + 1;
      }
      if (add) kept += (kept.length() ? ";" : "") + key;
      cfg.ignore = kept;
      save_settings();
      log_event(String(add ? "ignoring" : "no longer ignoring") + " a private-address device");
    }
    load_ignores();
    String r = String("guests ") + (cfg.guests ? "on" : "off") + "\n";
    xSemaphoreTake(presence_mutex, portMAX_DELAY);
    for (int i = 0; i < guest_count; i++)
      r += mac_text(guests[i].mac) + " last seen " + String((millis() - guests[i].seen_ms) / 60000UL) + " min ago at " +
           IPAddress(guests[i].ip).toString() + "\n";
    for (int k = 0; k < ignored_count; k++) r += mac_text(ignored[k]) + " ignored\n";
    xSemaphoreGive(presence_mutex);
    return r;
  }
  if (verb == "hue") {
    if (rest.startsWith("pair ")) return hue_pair(rest.substring(5));
    if (rest == "forget") { cfg.hue = ""; save_settings(); hue_reload = true; return "all Hue bridges forgotten"; }
    if (rest.startsWith("forget ")) { hue_set_bridge(rest.substring(7), ""); return "forgot the Hue bridge at " + rest.substring(7); }
    if (rest == "batteries") { hue_batt_pending = bridge_count > 0; return hue_batteries_text() + "(reading them again now)"; }
    if (rest.startsWith("delete-light ")) return hue_delete_light(rest.substring(13));
    if (rest == "lights") { hue_batt_pending = bridge_count > 0; return hue_lights_text() + "(reading them again now)"; }
    return hue_bridges_text();
  }
  if (verb == "upload") {
    if (rest == "on" || rest == "off") { cfg.upload = rest == "on"; save_settings(); }
    return String("upload ") + (cfg.upload ? "on" : "off");
  }
  if (verb == "update") { update_check_requested = true; return "checking for new firmware"; }
  if (verb == "reboot") { stop_run("reboot"); log_event("rebooting on command"); delay(300); ESP.restart(); return ""; }
  return "run | stop | mode auto|home|away | set ... | phone add|del ... | phones | guests [on|off] | guest ignore|unignore <mac> | hue [pair <ip>|forget [ip]|batteries|lights|delete-light <b.id>] | upload on|off | update check | reboot";
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
  s += "  \"guest_min_ago\": " + minutes_since(last_guest_ms()) + ",\n";
  s += "  \"guest_arrival_min_ago\": " + minutes_since(last_guest_arrival_ms()) + ",\n";
  s += "  \"guest_devices\": " + String(guests_present()) + ",\n";
  s += "  \"hue_change_min_ago\": " + minutes_since(hue_change_ms) + ",\n";
  s += "  \"hue_lights_on\": " + String(hue_lights_on) + ",\n";
  s += "  \"hue_bridges\": " + String(bridge_count) + ",\n";
  int low = -1;
  xSemaphoreTake(hue_mutex, portMAX_DELAY);
  for (int i = 0; i < hue_batt_count; i++) if (low < 0 || hue_batts[i].s.battery < low) low = hue_batts[i].s.battery;
  xSemaphoreGive(hue_mutex);
  s += "  \"hue_lowest_battery\": " + String(low) + ",\n";
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

// Up to two networks are compiled in (rcctl wifi): normally the house's, plus a second for bench testing
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
  hue_mutex = xSemaphoreCreateMutex();
  recent_lines.init(32 * 1024);
  upload_queue.init(512 * 1024);            // In PSRAM: days of telemetry if bedichek.org is unreachable
  load_settings();
  load_phones();
  load_ignores();
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
