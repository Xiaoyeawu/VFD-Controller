# VFD Controller (Tekovate)

Matter "virtual VFD" controller for a 3-phase induction motor, running on a
**Seeed Studio XIAO ESP32-C6**. Pairs to Apple Home and Aqara over Matter,
exposes motor on/off + speed, a WiFi status LED, a 5-second factory-reset
button, and a GitHub-based OTA update system.

> **Phase 1 (now):** no real VFD attached — speed (Hz / RPM) is computed and
> printed to Serial. **Phase 2 (later):** the same value is pushed to a real
> VFD over RS485 Modbus RTU. Phase 2 hooks are stubbed and guarded by `PHASE2`.

## Matter endpoints (one device, three endpoints)

| Endpoint | Type | App label | Function |
|----------|------|-----------|----------|
| 1 | Dimmable light | **VFD Motor** | on/off = run/stop; brightness 0–100% = 0–50 Hz / 0–1500 RPM |
| 2 | On/Off plug | **Firmware Update** | turn ON to trigger a GitHub OTA; auto-resets to OFF |
| 3 | Contact sensor | **Update Available** | "detected" when a newer firmware version exists |

## Build settings (Arduino IDE)

- **Board:** XIAO_ESP32C6
- **Partition Scheme:** Huge APP (3MB No OTA/1MB SPIFFS) — required, Matter is large
- **Erase All Flash Before Sketch Upload:** Enable (first flash only)

No WiFi credentials live in the source. On the ESP32-C6, Matter commissions
over BLE and the Home app supplies WiFi credentials during pairing.

## Status LED (GPIO15, active-LOW)

| State | LED |
|-------|-----|
| Not commissioned | slow blink (~1 Hz) |
| WiFi connecting | medium blink (~2.5 Hz) |
| Connected (normal) | solid on |
| WiFi lost | fast blink (~5 Hz) |
| OTA in progress | very fast strobe |
| BOOT button held | off (reset feedback) |

## Factory reset

Hold the **BOOT button (GPIO9)** for 5 seconds. The Matter fabric + WiFi are
wiped and the device reboots into pairing mode. Short presses are ignored.

## Releasing a firmware update (OTA)

1. Bump `currentVersion` in `VFD_Controller.ino` (e.g. `1.0.0.1` → `1.0.0.2`).
2. Arduino IDE → **Sketch ▸ Export Compiled Binary** (same partition scheme).
3. Rename the exported `*.ino.bin` to **`firmware.bin`** and upload it to this
   repo's `main` branch.
4. Update **`version.txt`** to the new version string and commit.
5. In the Home app, turn the **Firmware Update** plug ON — the device checks,
   downloads, flashes, and reboots on the new version.

OTA reads:
- `https://raw.githubusercontent.com/Xiaoyeawu/VFD-Controller/main/version.txt`
- `https://raw.githubusercontent.com/Xiaoyeawu/VFD-Controller/main/firmware.bin`

## Phase 2 — RS485 wiring (not active yet)

```
XIAO ESP32-C6      MAX485        VFD terminal
3V3            ->  VCC
GND            ->  GND
GPIO16 (D6)    ->  DI
GPIO17 (D7)    ->  RO
GPIO20 (D10)   ->  DE + RE (tied)
                   A    ->       485+
                   B    ->       485-
```
