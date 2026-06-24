// =====================================================================
//  VFD Controller  -  Matter "virtual VFD" for a 3-phase induction motor
//  Board   : Seeed Studio XIAO ESP32-C6
//  Core    : Arduino-ESP32 v3.3.x   (Espressif built-in Matter.h)
//  Author  : Kelvin / Tekovate
//
//  PHASE 1 (this build): Matter device, no real VFD attached. Speed is
//  computed and printed to Serial. Pairs to Apple Home + Aqara.
//  PHASE 2 (future): same speed value is pushed to a real VFD over
//  RS485 Modbus RTU (MAX485). Hooks are stubbed below, guarded by PHASE2.
//
//  WiFi: there is NO hardcoded SSID/password anywhere. On the ESP32-C6,
//  Matter commissions over BLE and the Home app delivers WiFi credentials
//  during pairing (see the #if !CONFIG_ENABLE_CHIPOBLE block in setup()).
//
//  Arduino IDE build settings (REQUIRED):
//    Board            : XIAO_ESP32C6
//    Partition Scheme : Huge APP (3MB No OTA/1MB SPIFFS)   <-- Matter is large
//    Erase All Flash Before Sketch Upload : Enable (first flash only)
// =====================================================================

#include <Matter.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Update.h>

// ---------------------------------------------------------------------
//  VERSION + GITHUB OTA ENDPOINTS
//  Bump currentVersion before every release, then upload the matching
//  firmware.bin + an updated version.txt to the repo's main branch.
// ---------------------------------------------------------------------
String currentVersion = "1.0.0.2";   // <-- bump on each new release

const char* VERSION_URL = "https://raw.githubusercontent.com/Xiaoyeawu/VFD-Controller/main/version.txt";
const char* OTA_URL     = "https://raw.githubusercontent.com/Xiaoyeawu/VFD-Controller/main/firmware.bin";

// ---------------------------------------------------------------------
//  PINS  (XIAO ESP32-C6)
// ---------------------------------------------------------------------
#define LED_PIN       15      // onboard user LED  -- ACTIVE LOW
#define LED_ON        LOW     //   write LOW  = LED on
#define LED_OFF       HIGH    //   write HIGH = LED off

#define BOOT_BTN_PIN  9       // BOOT button -- ACTIVE LOW (pressed = LOW)
#define RESET_HOLD_MS 5000    // hold 5 s for factory reset

// ---- PHASE 2 RS485 pins (documented now, wired later) ----------------
#define RS485_DI_PIN  16      // D6  -> MAX485 DI   (TX)
#define RS485_RO_PIN  17      // D7  -> MAX485 RO   (RX)
#define RS485_DE_PIN  20      // D10 -> MAX485 DE + RE (tied together)

// ---------------------------------------------------------------------
//  PHASE 2 SWITCH
//    0 = Matter only (no RS485 traffic) -- safe default, ships like this
//    1 = also drive the real VFD over RS485 Modbus RTU (wire MAX485 first)
// ---------------------------------------------------------------------
#define PHASE2 0

// ---------------------------------------------------------------------
//  PHASE 2: VFD Modbus settings  --  VERIFY AGAINST YOUR VFD MANUAL
//  Drive: ZESTABLE ZA280A-1.5G3 (Jinqu Electric). No public datasheet was
//  found, so these use the convention used by the large majority of generic
//  Chinese VFDs. Confirm on the bench (watch the VFD keypad) before trusting
//  them with a spinning motor.
//
//    Frequency setpoint (0x1000): signed -10000..10000 = -100.00%..100.00%
//      of the VFD's configured MAX frequency. So value = hz / VFD_MAX_HZ * 10000.
//    Command (0x2000): 1=fwd run, 2=rev, 3=fwd jog, 4=rev jog,
//      5=decel stop, 6=coast stop, 7=fault reset.
// ---------------------------------------------------------------------
#define VFD_SLAVE_ADDR   1        // RS485 station address (set on the VFD's comm param)
#define VFD_BAUD         9600     // 9600 8N1
#define VFD_REG_FREQ     0x1000   // frequency setpoint register (percent of max)
#define VFD_REG_CMD      0x2000   // run/stop command register
#define VFD_CMD_RUN_FWD  0x0001   // forward run
#define VFD_CMD_STOP     0x0005   // decelerate to stop
#define VFD_MAX_HZ       50.0f    // MUST equal the VFD's configured max frequency

// ---------------------------------------------------------------------
//  MATTER ENDPOINTS  (one node, three endpoints)
//   1) Dimmable light  -> "VFD Motor"        on/off = run/stop, brightness = speed
//   2) On/Off plug     -> "Firmware Update"   turn ON to trigger a GitHub OTA
//   3) Contact sensor  -> "Update Available"  flips when a newer version exists
// ---------------------------------------------------------------------
MatterDimmableLight VFDMotor;
MatterOnOffPlugin   FirmwareUpdatePlug;
MatterContactSensor UpdateAvailable;

// ---------------------------------------------------------------------
//  GLOBAL STATE
// ---------------------------------------------------------------------
volatile bool otaRequested  = false;   // set by the Firmware Update plug callback
bool          otaInProgress = false;   // true while flashing (drives LED strobe)
bool          buttonHeld    = false;   // true while BOOT button is being held
bool          wasConnected  = false;   // have we ever reached WL_CONNECTED?

// VFD target state. The Matter callbacks just update these (fast, non-blocking);
// the actual RS485 write happens throttled in loop() so dragging the speed
// slider can't flood the Modbus bus.
volatile bool  g_motorOn  = false;
volatile float g_targetHz = 0.0f;
volatile bool  g_vfdDirty = true;      // a change is pending push to the VFD

unsigned long wifiConnectedAt = 0;     // millis() when WiFi first came up
bool          bootCheckDone   = false; // one-shot ~10 s post-connect version check

unsigned long lastOTACheck = 0;
const unsigned long OTA_CHECK_INTERVAL = 3600000UL;   // 1 hour

// Status-LED modes. Declared up here (before the first function) so the
// Arduino IDE's auto-generated prototypes can see the type -- otherwise
// the prototype for setLED(LedMode) lands above this declaration and fails.
enum LedMode { LED_M_OFF, LED_M_SOLID, LED_M_SLOW, LED_M_MEDIUM, LED_M_FAST, LED_M_STROBE };

// =====================================================================
//  SPEED MAPPING  -- single source of truth
//    brightness 0..254  ->  percent 0..100  ->  Hz 0..50  ->  RPM 0..1500
//    (RPM assumes a 4-pole motor: rpm = hz * 30)
// =====================================================================
void computeSpeed(uint8_t brightness, int &percent, float &hz, int &rpm) {
  percent = (int)(brightness / 2.54f);          // 0..100 (display)
  hz      = (brightness / 254.0f) * 50.0f;       // 0..50 Hz
  rpm     = (int)(hz * 30.0f);                    // 0..1500 RPM
}

// =====================================================================
//  STATUS LED  -- non-blocking, millis-based multi-mode helper
//  (pattern adapted from the Tropika reference sketch's setLED())
//  (LedMode enum is declared earlier, before the first function.)
// =====================================================================
void setLED(LedMode mode) {
  switch (mode) {
    case LED_M_OFF:   digitalWrite(LED_PIN, LED_OFF); return;
    case LED_M_SOLID: digitalWrite(LED_PIN, LED_ON);  return;
    default: break;
  }
  uint16_t halfPeriod;
  switch (mode) {
    case LED_M_SLOW:   halfPeriod = 1000; break;  // ~1   Hz : not commissioned
    case LED_M_MEDIUM: halfPeriod = 200;  break;  // ~2.5 Hz : WiFi connecting
    case LED_M_FAST:   halfPeriod = 100;  break;  // ~5   Hz : WiFi lost
    default:           halfPeriod = 50;   break;  // strobe  : OTA in progress
  }
  bool on = (millis() / halfPeriod) % 2;
  digitalWrite(LED_PIN, on ? LED_ON : LED_OFF);
}

bool netUp() {
  // IMPORTANT: under Matter BLE commissioning the CHIP stack (not the Arduino
  // WiFi object) brings up and owns the WiFi connection. As a result
  // WiFi.status() reads disconnected and WiFi.localIP() reads 0.0.0.0 even
  // when the device is fully online (HomeKit/Aqara can control it over IP).
  // Matter's own connectivity flag is the authoritative "online" signal.
  return Matter.isWiFiConnected();
}

void updateStatusLED() {
  if (otaInProgress)               { setLED(LED_M_STROBE); return; }   // OTA strobe
  if (buttonHeld)                  { setLED(LED_M_OFF);    return; }   // feedback during hold
  if (!Matter.isDeviceCommissioned()) { setLED(LED_M_SLOW); return; } // waiting to pair

  if (netUp()) {
    wasConnected = true;
    setLED(LED_M_SOLID);                         // normal: connected
  } else {
    setLED(wasConnected ? LED_M_FAST             // lost after being connected
                        : LED_M_MEDIUM);         // connecting for the first time
  }
}

// =====================================================================
//  FACTORY RESET  -- BOOT button held LOW for 5 s wipes the Matter fabric
//  Short presses are ignored (BOOT is also the flashing button).
// =====================================================================
void handleFactoryReset() {
  static unsigned long pressStart = 0;

  if (digitalRead(BOOT_BTN_PIN) == LOW) {        // pressed
    if (pressStart == 0) pressStart = millis();
    buttonHeld = true;
    if (millis() - pressStart >= RESET_HOLD_MS) {
      Serial.println("\nFACTORY RESET - clearing Matter fabric + WiFi");
      setLED(LED_M_OFF);
      Matter.decommission();                     // wipe Matter NVS / fabric
      delay(500);
      ESP.restart();
    }
  } else {                                        // released
    pressStart  = 0;
    buttonHeld  = false;
  }
}

// =====================================================================
//  GITHUB OTA
// =====================================================================

// checkVersion(): compare remote version.txt to currentVersion.
// Updates the "Update Available" contact sensor. Returns true if newer.
bool checkVersion() {
  if (!netUp()) {
    Serial.println("[OTA] checkVersion skipped - no network");
    return false;
  }

  Serial.println("[OTA] checking remote version...");
  WiFiClientSecure client;
  client.setInsecure();                          // public firmware: skip cert check

  HTTPClient https;
  bool newer = false;

  if (https.begin(client, VERSION_URL)) {
    int code = https.GET();
    if (code == HTTP_CODE_OK) {
      String remote = https.getString();
      remote.trim();
      Serial.printf("[OTA] remote=%s  current=%s\n", remote.c_str(), currentVersion.c_str());
      newer = (remote.length() > 0 && remote != currentVersion);
    } else {
      Serial.printf("[OTA] version check HTTP error: %d\n", code);
    }
    https.end();
  } else {
    Serial.println("[OTA] version check begin() failed");
  }

  UpdateAvailable.setContact(newer);             // detected = update available
  Serial.println(newer ? "[OTA] UPDATE AVAILABLE" : "[OTA] firmware up to date");
  return newer;
}

// doOTAUpdate(): download firmware.bin and flash it, then reboot.
void doOTAUpdate() {
  if (!netUp()) {
    Serial.println("[OTA] update aborted - no network");
    return;
  }

  otaInProgress = true;
  setLED(LED_M_STROBE);                           // visible before the blocking download
  Serial.println("[OTA] downloading firmware.bin ...");

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;
  if (https.begin(client, OTA_URL)) {
    int code = https.GET();
    if (code == HTTP_CODE_OK) {
      int len = https.getSize();
      Serial.printf("[OTA] content length: %d bytes\n", len);

      if (len > 0 && Update.begin(len)) {
        WiFiClient *stream = https.getStreamPtr();
        size_t written = Update.writeStream(*stream);
        Serial.printf("[OTA] written %u / %d bytes\n", (unsigned)written, len);

        if (Update.end()) {
          if (Update.isFinished()) {
            Serial.println("[OTA] SUCCESS - rebooting into new firmware...");
            delay(500);
            ESP.restart();
          } else {
            Serial.println("[OTA] update not finished (incomplete stream)");
          }
        } else {
          Serial.printf("[OTA] Update.end() error #%d\n", Update.getError());
        }
      } else {
        Serial.println("[OTA] Update.begin() failed or bad content length");
      }
    } else {
      Serial.printf("[OTA] firmware HTTP error: %d\n", code);
    }
    https.end();
  } else {
    Serial.println("[OTA] firmware begin() failed");
  }

  otaInProgress = false;                          // only reached on failure
}

// =====================================================================
//  PHASE 2: real VFD over RS485 Modbus RTU  (MAX485)  -- STUBS ONLY
// ---------------------------------------------------------------------
//  Wiring:
//    XIAO ESP32-C6      MAX485        VFD terminal
//    3V3            ->  VCC
//    GND            ->  GND
//    GPIO16 (D6)    ->  DI
//    GPIO17 (D7)    ->  RO
//    GPIO20 (D10)   ->  DE + RE (tied)
//                       A    ->       485+
//                       B    ->       485-
//
//  Register/coil addresses depend on the specific VFD (user supplies the
//  manual in Phase 2). Do NOT implement the writes yet.
// =====================================================================
#if PHASE2
// Standard Modbus RTU CRC-16 (poly 0xA001). Returned low-byte-first on the wire.
static uint16_t modbusCRC(const uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
      else                crc >>= 1;
    }
  }
  return crc;
}

// Modbus function 0x06 -- write single holding register. Best-effort read of
// the echo response (0x06 echoes the request frame). Returns true if any
// response arrived.
bool vfdWriteReg(uint16_t reg, uint16_t value) {
  uint8_t f[8];
  f[0] = VFD_SLAVE_ADDR;
  f[1] = 0x06;
  f[2] = reg   >> 8;  f[3] = reg   & 0xFF;
  f[4] = value >> 8;  f[5] = value & 0xFF;
  uint16_t crc = modbusCRC(f, 6);
  f[6] = crc & 0xFF;  f[7] = crc >> 8;     // CRC low byte first

  digitalWrite(RS485_DE_PIN, HIGH);        // enable driver -> transmit
  delayMicroseconds(50);
  Serial1.write(f, 8);
  Serial1.flush();                         // block until the last byte is shifted out
  delayMicroseconds(100);                  // let the stop bit clear the line
  digitalWrite(RS485_DE_PIN, LOW);         // back to receive

  Serial.printf("[VFD] TX fc06 reg=0x%04X val=%u\n", reg, value);

  unsigned long t0 = millis();
  uint8_t n = 0, resp[8];
  while (millis() - t0 < 100 && n < sizeof(resp)) {
    if (Serial1.available()) resp[n++] = (uint8_t)Serial1.read();
  }
  if (n == 0) {
    Serial.println("[VFD] no response (check wiring / slave addr / baud / VFD power)");
    return false;
  }
  return true;
}

void vfdInit() {
  Serial1.begin(VFD_BAUD, SERIAL_8N1, RS485_RO_PIN, RS485_DI_PIN);
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);         // receive mode by default
  Serial.printf("[VFD] Modbus RTU ready on Serial1 (%d 8N1), slave %d\n",
                VFD_BAUD, VFD_SLAVE_ADDR);
}

void vfdWriteFrequency(float hz) {
  if (hz < 0)          hz = 0;
  if (hz > VFD_MAX_HZ) hz = VFD_MAX_HZ;
  uint16_t val = (uint16_t)lroundf(hz / VFD_MAX_HZ * 10000.0f);  // 0..10000 = 0..100%
  vfdWriteReg(VFD_REG_FREQ, val);
}

void vfdRun(bool on) {
  vfdWriteReg(VFD_REG_CMD, on ? VFD_CMD_RUN_FWD : VFD_CMD_STOP);
}
#endif

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println(" VFD Controller (Tekovate)  -  Matter");
  Serial.printf (" Firmware version: %s\n", currentVersion.c_str());
  Serial.println("========================================");

  pinMode(LED_PIN, OUTPUT);
  setLED(LED_M_OFF);
  pinMode(BOOT_BTN_PIN, INPUT_PULLUP);

#if PHASE2
  vfdInit();
#endif

  // ---- WiFi -----------------------------------------------------------
  // When BLE commissioning is enabled (ESP32-C6 default), WiFi credentials
  // are delivered by the Home app during pairing -- nothing to set here.
  // The block below only compiles in if BLE commissioning is unavailable,
  // and even then we keep the spec's promise of NO credentials in source.
#if !CONFIG_ENABLE_CHIPOBLE
  Serial.println("[WiFi] BLE commissioning unavailable on this build.");
  Serial.println("[WiFi] WiFi credentials must be provided another way.");
  // Intentionally NOT calling WiFi.begin() with hardcoded credentials.
#endif

  // ---- Matter endpoints ----------------------------------------------
  VFDMotor.begin();                       // dimmable light  : VFD Motor
  FirmwareUpdatePlug.begin(false);        // on/off plug     : Firmware Update
  UpdateAvailable.begin(false);           // contact sensor  : Update Available

  // Endpoint 1: VFD Motor  -- run/stop
  VFDMotor.onChangeOnOff([](bool on) -> bool {
    Serial.println(on ? ">> MOTOR RUN" : ">> MOTOR STOP");
    g_motorOn  = on;
    g_vfdDirty = true;        // pushed to the VFD in loop() (Phase 2)
    return true;
  });

  // Endpoint 1: VFD Motor  -- speed (brightness)
  VFDMotor.onChangeBrightness([](uint8_t brightness) -> bool {
    int percent, rpm; float hz;
    computeSpeed(brightness, percent, hz, rpm);
    Serial.printf(">> SPEED: %d%%  ->  %.1f Hz  ->  %d RPM\n", percent, hz, rpm);
    g_targetHz = hz;
    g_vfdDirty = true;        // pushed to the VFD in loop() (Phase 2)
    return true;
  });

  // Endpoint 2: Firmware Update plug -- ON triggers a GitHub OTA.
  // We only set a flag here; the work happens in loop() so we never block
  // a Matter callback (and the plug is reset to OFF afterwards).
  FirmwareUpdatePlug.onChangeOnOff([](bool on) -> bool {
    if (on) {
      Serial.println("[OTA] Firmware Update plug switched ON -> request queued");
      otaRequested = true;
    }
    return true;
  });

  // ---- Start Matter ---------------------------------------------------
  Matter.begin();

  // NOTE on device identity (Manufacturer/Model/Serial in section 10):
  // the Basic Information cluster strings are CHIP compile-time defaults in
  // this Arduino core and have no runtime setter. To fully customize them
  // ("Tekovate" / "VFD-CTRL-C6" / "001") requires an esp-matter factory
  // partition / menuconfig build. Phase 1 ships with the core defaults.

  if (!Matter.isDeviceCommissioned()) {
    Serial.println("\n=====================================================");
    Serial.println(" NOT COMMISSIONED - pair this device now.");
    Serial.printf ("  Manual pairing code : %s\n", Matter.getManualPairingCode().c_str());
    Serial.printf ("  QR code URL         : %s\n", Matter.getOnboardingQRCodeUrl().c_str());
    Serial.println("=====================================================\n");
  } else {
    Serial.println("[Matter] Device already commissioned.");
  }
}

// =====================================================================
//  LOOP
// =====================================================================
void loop() {
  // 1) Status LED + factory-reset button (both non-blocking)
  handleFactoryReset();
  updateStatusLED();

  // 1b) Throttled status heartbeat (every 5 s) so on-board state is visible
  static unsigned long lastBeat = 0;
  if (millis() - lastBeat > 5000) {
    lastBeat = millis();
    // online = Matter's authoritative WiFi flag. arduino_ip usually reads
    // 0.0.0.0 under Matter-managed WiFi -- that's expected, not a fault.
    Serial.printf("[status] commissioned=%d  online=%d  arduino_ip=%s\n",
                  Matter.isDeviceCommissioned() ? 1 : 0,
                  netUp() ? 1 : 0,
                  WiFi.localIP().toString().c_str());
  }

  // 1c) Phase 2: push the latest run/speed to the real VFD, throttled to
  //     10 Hz so dragging the slider can't flood the RS485 bus. Disabled
  //     (compiled out) until PHASE2 is set to 1 and the MAX485 is wired.
#if PHASE2
  static unsigned long lastVfdPush = 0;
  if (g_vfdDirty && (millis() - lastVfdPush > 100)) {
    lastVfdPush = millis();
    g_vfdDirty  = false;
    vfdRun(g_motorOn);
    vfdWriteFrequency(g_motorOn ? g_targetHz : 0.0f);
  }
#endif

  // 2) Track WiFi connection for the boot-time version check
  if (netUp()) {
    if (wifiConnectedAt == 0) {
      wifiConnectedAt = millis();
      Serial.println("[net] Matter WiFi connected (online)");
    }
  } else {
    wifiConnectedAt = 0;
    bootCheckDone   = false;
  }

  // 3) Boot check: ~10 s after WiFi comes up, set the Update Available sensor
  if (netUp() && !bootCheckDone && wifiConnectedAt != 0 &&
      (millis() - wifiConnectedAt > 10000)) {
    bootCheckDone = true;
    lastOTACheck  = millis();
    checkVersion();
  }

  // 4) Hourly auto-check (compare only, no download)
  if (netUp() && bootCheckDone && (millis() - lastOTACheck > OTA_CHECK_INTERVAL)) {
    lastOTACheck = millis();
    checkVersion();
  }

  // 5) App-triggered OTA: Firmware Update plug was switched ON
  if (otaRequested) {
    otaRequested = false;
    Serial.println("[OTA] handling Firmware Update request...");
    if (checkVersion()) {
      doOTAUpdate();                 // downloads, flashes, reboots on success
    } else {
      Serial.println("[OTA] nothing to do - already on latest version");
    }
    FirmwareUpdatePlug.setOnOff(false);   // auto-reset the plug back to OFF
  }

  delay(20);   // small yield; keeps LED timing smooth without busy-looping
}
