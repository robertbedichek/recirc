# recirc

Controller for the hot-water recirculation pump at the house: a Waveshare
[ESP32-S3-Relay-1CH](https://www.waveshare.com/wiki/ESP32-S3-Relay-1CH) on Wi-Fi, switching a Grundfos
120 VAC pump (about 30 W).

## What it does

- Every 20 minutes, on the clock (:00, :20, :40), it runs the pump until the return line has warmed 4 °F above
  where it started, or for 80 seconds, whichever comes first.
- It runs only when someone is home, and never from 11 PM to 7 AM.
- The run-now button runs it at any time.
- It never runs without a working return-line temperature.
- **Someone is home** when one of these happened within the last 60 minutes:
  - A family phone was seen on the house Wi-Fi. Every 2 minutes it sweeps the local /24 for their MAC
    addresses using ARP, the network's address lookup.
  - Any other device with a private (randomized) Wi-Fi address was seen in the same sweep. Phones, including
    guests', join with private addresses, while routers, Hue bridges, cameras and computers left on use their
    real ones, so this counts visitors without registering them. `guests off` turns it off; `guest ignore <mac>`
    excludes a private-address device that is always there (an iPad left at the cabin, say). `guests` lists
    what it has seen.
  - A Hue light was switched or dimmed. It polls the bridge's local API every minute.

  With all of these off it assumes someone is home. `mode home` and `mode away` override this.
- **Time** comes from NTP and is kept across outages by the onboard PCF85063 clock chip, which holds UTC.
  The schedule uses Pacific time.
- **Telemetry** is posted to `https://bedichek.org/home/api/ingest/recirc`, appending to
  `/var/www/html/home/recirc.txt`. Lines containing "alert" go to Pushover, as for the solar controllers.
  - `YYYY-MM-DD HH:MM:SS run schedule warm secs 34 start 61.2 end 65.4 peak 65.6`
  - `YYYY-MM-DD HH:MM:SS status T 64.2 home 1 phone 12 guest 3 devices 2 hue 45 lights 3/41 runs 14 secs 410 rssi -62`,
    every 5 minutes. `phone`, `guest` and `hue` are minutes since last seen, -1 meaning never; `devices` is how
    many guest devices were seen within the hold time.
  - `YYYY-MM-DD HH:MM:SS arrive guest 1 present 2` when a family phone or guest device shows up after being away
    longer than the hold time. recirc.txt is public, so these lines carry no MAC addresses or names.
- **Firmware updates are pulled**, because the house LAN isn't reachable from Palo Alto.
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

1. `./rcctl wifi`: enter the cabin Wi-Fi name and password, and optionally a second network (e.g. your home network, for
   testing in Palo Alto). They're stored in `~/.config/recirc/wifi`, not in git. The board joins whichever of
   the two it can see, preferring the cabin's, and looks again after 3 minutes without Wi-Fi.
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
   - Set each family iPhone's cabin Wi-Fi to **Private Wi-Fi Address: Fixed**.
   - Read each phone's address from Settings → Wi-Fi → (network) → Wi-Fi Address.
   - `./rcctl cmd phone add aa:bb:cc:dd:ee:ff rob`
5. Pair Hue: press the bridge's round link button, then within 30 s run `./rcctl cmd hue pair <bridge ip>`.
6. After that: `./rcctl publish`, and the board takes the new build within the hour.
   `./rcctl cmd update check` makes it look now.

## Commands

`run`, `stop`, `mode auto|home|away`, `set interval|maxrun|rise|hold|quiet <v>`, `phone add|del`, `phones`,
`guests [on|off]`, `guest ignore|unignore <mac>`,
`hue pair <ip>|forget`, `upload on|off`, `update check`, `reboot`. Send them over USB serial, or with
`./rcctl cmd ...` on the LAN.
