# recirc

Controller for a house's hot-water recirculation pump: a Waveshare
[ESP32-S3-Relay-1CH](https://www.waveshare.com/wiki/ESP32-S3-Relay-1CH) on Wi-Fi, switching a Grundfos
120 VAC pump (about 30 W).

## What it does

- Every 20 minutes, on the clock (:00, :20, :40), it runs the pump until the return line has warmed 4 °F above
  where it started, or for 80 seconds, whichever comes first.
- It runs only when someone is home, and never from 11 PM to 7 AM.
- The run-now button runs it at any time.
- It never runs without a working return-line temperature.
- **Someone is home** when one of these holds:
  - A family phone was seen on the house Wi-Fi within the last 60 minutes (`hold`). Every 2 minutes it sweeps
    the local /24 for their MAC addresses using ARP, the network's address lookup.
  - A Hue light was switched or dimmed within the last 2 hours (`huehold`). It polls each paired bridge's local
    API every minute (up to four bridges). Brightness changes on lights that are off, and polls where the number
    of lights differs (a short read), don't count.
  - A guest device arrived within the last 2 hours (`guesthold`): any other device with a private (randomized)
    Wi-Fi address, seen in the same sweep after being gone 60 minutes or more. Phones, including guests', join
    with private addresses, while routers, Hue bridges, cameras and computers left on use their real ones, so
    this counts visitors without registering them. It counts arrivals rather than presence because a house
    collects private-address devices that never leave; devices seen in the first 10 minutes after boot were
    already there. `guests off` turns it off; `guest ignore <mac>` drops a device entirely. `guests` lists what
    it has seen.
  - **An occupancy signal** within `occhold` minutes (120): any GET or POST of `http://<controller>/occupied`. An
    Apple Home automation can send it through a shortcut ("Get Contents of URL") when a room sensor, such as an
    ecobee SmartSensor, detects occupancy; Apple Home runs it on a home hub (Apple TV or HomePod). It takes no
    token, since all it can do is mark the house occupied for a while, and only from the local network. Give each
    sensor's automation its own number, `/occupied?room=3` (1-16), to see which rooms are in use: logged as
    `# occupancy signal room 3` at most once every 10 minutes per room. Names for the numbers belong on the private
    page, not in the log.

  With all of these off it assumes someone is home. `mode home` and `mode away` override this.
- **Time** comes from NTP and is kept across outages by the onboard PCF85063 clock chip, which holds UTC.
  The schedule uses Pacific time.
- **Telemetry** is posted to `https://bedichek.org/home/api/ingest/recirc`, appending to
  `recirc.txt` in a private, unlisted folder on the server (its path is set in the ingest service, not here). Lines containing "alert" go to Pushover, as for the solar controllers.
  - `YYYY-MM-DD HH:MM:SS run schedule warm secs 34 start 61.2 end 65.4 peak 65.6`
  - `YYYY-MM-DD HH:MM:SS status T 64.2 home 1 phone 12 guest 3 arrive 40 devices 2 hue 45 lights 3/41 runs 14 secs 410 rssi -62 occ 8`,
    every 5 minutes (`occ`: minutes since an occupancy signal). `phone`, `guest` and `hue` are minutes since last seen, `arrive` minutes since a guest device
    arrived, -1 meaning never; `devices` is how many guest devices were seen within the hold time.
  - `YYYY-MM-DD HH:MM:SS arrive guest 1 present 2` when a family phone or guest device shows up after being away
    longer than the hold time. These lines carry no MAC addresses or names.
  - `YYYY-MM-DD HH:MM:SS hue battery 143.2 87 reachable 1 model RWL022 fw 2.45.2_hF4400CA`, once after boot and then daily at 09:00,
    for every battery-powered Hue accessory (dimmers, buttons, tap dials, motion sensors), and at the hourly check for
    any whose % moved 20 points or more since the last reading (so a new cell shows within the hour). `143.2` is the
    bridge's last address byte and the bridge's id for the accessory; `hue batteries` shows the names, which
    stay out of recirc.txt. A v2 dimmer's % runs from about 3.0 V (0%) to 3.25 V (100%), and it quits a little under 3 V.
    It also checks every hour, without logging, and alerts when an accessory that was reachable stops responding.
  - `YYYY-MM-DD HH:MM:SS hue light 143.3 maker Signify_Netherlands_B.V. model LCA001 fw 1.122.2 reachable 1`: each
    bulb's maker, model and firmware (spaces become `_`), at the first reading after boot and whenever one of
    those changes. `hue lights` lists them with names.
  - `YYYY-MM-DD HH:MM:SS hue radio 143 zigbee 25 wifi 6` at the same time: each bridge's Zigbee channel and the
    2.4 GHz channel of the access point the controller is on, to check them for overlap.
  - `YYYY-MM-DD HH:MM:SS hue bridge 143 model BSB002 fw 1978688000 api 1.78.0 auto 1 update noupdates installed 2026-11-12T10:05:22`
    at the same time: each bridge's model, firmware, API version, whether automatic updates are on (`auto` 1 or 0),
    its update state and when it last installed one. Checked hourly as well: a firmware change is logged as an event
    (`Hue bridge 143 firmware changed A -> B`), and so is automatic updates being switched; switching them on is an
    alert, for anyone who keeps them off and updates by hand.
  - The server's 6 AM snapshot script copies the `hue` lines into `recirc_hue.txt`, kept indefinitely, for the
    battery chart on the private page.
- **Firmware updates are pulled**, because the house LAN isn't reachable from outside.
  - Every hour it reads `recirc.json` from `https://bedichek.org/home/api/fw/recirc/` and, if the build
    differs, downloads `recirc.bin`, checks its SHA-256 and installs it.
  - A new image that can't reach bedichek.org within 2 minutes rolls back to the previous one.
  - `rcctl update` pushes over the LAN when you're at the house.

## Wiring

| Board | Goes to |
|---|---|
| Relay COM / NO | Pump hot (line) wire, in and out. Neutral and ground go straight to the pump. |
| VCC / GND screws | 5 V DC (Mean Well HDR-15-5 on the DIN rail, or USB-C) |
| 4-pin header, black (GND) | DS18B20 GND; run-now button |
| red (3V3) | DS18B20 VDD; 4.7 k pull-up |
| yellow (IO1) | DS18B20 data; other end of the 4.7 k |
| orange (IO2) | run-now button (to GND; internal pull-up) |

Clamp the DS18B20 to the return line near the water heater, with insulation over it. Onboard: relay on IO47,
the PCF85063 clock chip on I2C SDA 39, SCL 38 (address 0x51), RS485 on 17/18/21 (unused).

## Bring-up

1. `./rcctl wifi`: enter the house's Wi-Fi name and password, and optionally a second network (e.g. for
   bench testing elsewhere). They're stored in `~/.config/recirc/wifi`, not in git. The board joins whichever of
   the two it can see, preferring the first, and looks again after 3 minutes without Wi-Fi.
2. Plug the board into this Mac by USB-C, then run `./rcctl flash`. The board has 16 MB of flash (esptool,
   2026-11-13); the build uses two 3 MB update slots. Over-the-air updates can't change that layout.
3. Server side, done 2026-11-10:
   - `recirc` is in `/etc/home-ingest/tokens`, with home-ingest restarted.
   - `/var/lib/recirc-fw` is mode 2750 `claude:www-data`. rcctl publish scp's into it as `claude`.
   - `/etc/nginx/snippets/recirc-fw.conf`, included after `home-ingest.conf`, serves it at
     `/home/api/fw/recirc/` only with `Authorization: Bearer <fw_token>`. The firmware has the Wi-Fi
     password compiled in, so it must never be public.
   - Tested: the right token gets 200, no token or a wrong one gets 403, and the other `/home/api/` paths
     still reach the ingest service.
4. `./rcctl cmd upload on`. Then register the phones:
   - Set each family iPhone's house Wi-Fi to **Private Wi-Fi Address: Fixed**.
   - Read each phone's address from Settings → Wi-Fi → (network) → Wi-Fi Address.
   - `./rcctl cmd phone add aa:bb:cc:dd:ee:ff rob`
5. Pair Hue: for each bridge, press its round link button, then within
   30 s run `./rcctl cmd hue pair <bridge ip>`. `./rcctl cmd hue` lists the paired bridges with their Zigbee
   channels.
6. After that: `./rcctl publish`, and the board takes the new build within the hour.
   `./rcctl cmd update check` makes it look now.

## Commands

`run`, `stop`, `mode auto|home|away`, `set interval|maxrun|rise|hold|huehold|guesthold|quiet <v>`, `phone add|del`, `phones`,
`guests [on|off]`, `guest ignore|unignore <mac>`,
`hue` (list bridges), `hue pair <ip>`, `hue forget [ip]`, `hue batteries`, `hue lights`, `hue rooms` (rooms and zones with their bulbs, and the rooms each switch acts on, from the bridges' rules), `hue delete-light <bridge>.<id>` (only an unreachable bulb), `hue delete-sensor <bridge>.<id>` (only an unreachable switch or sensor), `upload on|off`, `update check`, `reboot`. Send them over USB serial, or with
`./rcctl cmd ...` on the LAN.
