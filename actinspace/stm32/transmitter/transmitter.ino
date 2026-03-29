/*
 * ActInSpace - Hot-Plug I2C Sensor Ground Station
 * STM32F401CCU6 (WeAct Black Pill)
 *
 * Hot-pluggable modules via 4-pin magnetic connector (GND, VCC, SDA, SCL):
 *   AHT20    (0x38)    — Temperature & Humidity
 *   BMP280   (0x76/77) — Pressure & Altitude
 *   MPU6500  (0x68)    — 6-axis IMU (Accel + Gyro)
 *   GPS M10  (0x42)    — GNSS Position (u-blox I2C/DDC)
 *   QMC5883L (0x0D)    — Compass / Magnetometer
 *   SSD1306  (0x3C)    — 0.96" OLED Status Display
 *
 * Always connected:
 *   LoRa LR900  — UART1 (PA9=TX, PA10=RX) @ 57600 baud
 *   Buzzer      — PA0 (tone output)
 *
 * I2C1 bus: PB6=SCL, PB7=SDA — external 4.7k pull-ups required
 * The pull-ups must be on the STM32 board (not on sensor modules)
 * so the bus stays stable when no module is connected.
 *
 * On magnetic snap connect:  ascending two-tone beep (only sound)
 * On magnetic snap disconnect: no sound
 * OLED cycles through connected module data every 2 seconds
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>
#include <MAVLink.h>

// ═══════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════
#define BUZZER_PIN  PA0

// ═══════════════════════════════════════════════════════════════
//  I2C ADDRESSES
// ═══════════════════════════════════════════════════════════════
#define ADDR_AHT20    0x38
#define ADDR_BMP280   0x77  // Combo module BMP280 (AHT20+BMP280)
#define ADDR_MPU6500  0x68
#define ADDR_GPS      0x42  // u-blox M10 DDC
#define ADDR_QMC5883  0x0D
#define ADDR_OLED     0x3C

// ═══════════════════════════════════════════════════════════════
//  DISPLAY
// ═══════════════════════════════════════════════════════════════
#define SCREEN_W 128
#define SCREEN_H  64

// ═══════════════════════════════════════════════════════════════
//  TIMING (ms)
// ═══════════════════════════════════════════════════════════════
#define SCAN_INTERVAL     500  // I2C bus scan period
#define DISPLAY_INTERVAL 2000  // Rotate OLED page
#define TELEM_INTERVAL    100  // 10 Hz MAVLink telemetry

// ═══════════════════════════════════════════════════════════════
//  MPU6500 REGISTERS
// ═══════════════════════════════════════════════════════════════
#define MPU_WHO_AM_I      0x75
#define MPU_PWR_MGMT_1    0x6B
#define MPU_ACCEL_XOUT_H  0x3B
#define MPU_ACCEL_CONFIG  0x1C
#define MPU_GYRO_CONFIG   0x1B

// ═══════════════════════════════════════════════════════════════
//  QMC5883L REGISTERS
// ═══════════════════════════════════════════════════════════════
#define QMC_DATA_OUT   0x00
#define QMC_STATUS     0x06
#define QMC_CTRL1      0x09
#define QMC_SET_RESET  0x0B

// ═══════════════════════════════════════════════════════════════
//  MODULE INDEX
// ═══════════════════════════════════════════════════════════════
enum ModID { M_AHT = 0, M_BMP, M_MPU, M_GPS, M_QMC, M_OLED, MOD_N };

const char* const MOD_NAMES[] = {
  "AHT20", "BMP280", "MPU6500", "GPS M10", "COMPASS", "DISPLAY"
};

// ═══════════════════════════════════════════════════════════════
//  MODULE STATE
// ═══════════════════════════════════════════════════════════════
struct Module {
  bool connected;     // currently detected on bus
  bool wasConnected;  // state at previous scan
  bool inited;        // sensor configured after detection
  uint8_t missCount;  // consecutive scans where probe failed
};

#define DISCONNECT_THRESHOLD 3  // must miss 3 scans in a row (~1.5s) to disconnect

Module mod[MOD_N];

// ═══════════════════════════════════════════════════════════════
//  HARDWARE OBJECTS
// ═══════════════════════════════════════════════════════════════
Adafruit_SH1106G oled(SCREEN_W, SCREEN_H, &Wire, -1);
Adafruit_BMP280  bmp;
Adafruit_AHTX0   aht;
HardwareSerial   lora(PA10, PA9);  // UART1 RX, TX

// ═══════════════════════════════════════════════════════════════
//  SENSOR DATA
// ═══════════════════════════════════════════════════════════════
// AHT20
float ahtTemp  = 0, ahtHumid = 0;
// BMP280
float bmpTemp  = 0, bmpPress = 0, bmpAlt = 0;
// MPU6500
float accX = 0, accY = 0, accZ = 0;
float gyrX = 0, gyrY = 0, gyrZ = 0;
float roll = 0, pitch = 0, yaw = 0;
// GPS
float   gpsLat = 0, gpsLon = 0, gpsAlt = 0;
uint8_t gpsSats = 0, gpsFix = 0;
// QMC5883L
float   magHeading = 0;
int16_t magRaw[3]  = {0, 0, 0};

// ═══════════════════════════════════════════════════════════════
//  DISPLAY CYCLE STATE
// ═══════════════════════════════════════════════════════════════
uint8_t dispList[MOD_N];  // IDs of connected sensor modules (excl OLED)
uint8_t dispCount = 0;    // how many in the cycle
uint8_t dispPage  = 0;    // current page index

// ═══════════════════════════════════════════════════════════════
//  TIMERS & COUNTERS
// ═══════════════════════════════════════════════════════════════
uint32_t lastScan  = 0;
uint32_t lastDisp  = 0;
uint32_t lastTelem = 0;
uint32_t counter   = 0;

// ═══════════════════════════════════════════════════════════════
//  GPS NMEA PARSER STATE
// ═══════════════════════════════════════════════════════════════
char    nmeaBuf[100];
uint8_t nmeaIdx    = 0;
bool    nmeaActive = false;

// Team ID for ground station commands
#define TEAM_ID "1003"
bool telemetry_enabled = true;

// ═══════════════════════════════════════════════════════════════
//  I2C HELPERS
// ═══════════════════════════════════════════════════════════════

bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  uint8_t err = Wire.endTransmission(true);
  // 0 = ACK, 2 = NACK (not present), 4 = other error
  // Treat only 0 as success; any error means not found
  return err == 0;
}

void i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t i2cReadReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.read();
}

// Recover a stuck I2C bus by bit-banging SCL + STOP condition,
// then fully reinitialising the peripheral.
void i2cRecover() {
  // Disable the I2C peripheral so pins become GPIO
  Wire.flush();
  __HAL_RCC_I2C1_FORCE_RESET();
  delay(2);
  __HAL_RCC_I2C1_RELEASE_RESET();
  delay(2);

  // Bit-bang SCL to free a stuck slave
  pinMode(PB6, OUTPUT);       // SCL
  pinMode(PB7, INPUT_PULLUP); // SDA
  delayMicroseconds(20);
  for (int i = 0; i < 16; i++) {
    digitalWrite(PB6, LOW);
    delayMicroseconds(10);
    digitalWrite(PB6, HIGH);
    delayMicroseconds(10);
    if (digitalRead(PB7)) break;   // SDA released
  }
  // STOP condition: SDA LOW→HIGH while SCL HIGH
  pinMode(PB7, OUTPUT);
  digitalWrite(PB7, LOW);
  delayMicroseconds(10);
  digitalWrite(PB7, HIGH);
  delayMicroseconds(20);

  // Reinitialise Wire from scratch
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  Wire.setClock(100000);
  Wire.setTimeout(25);
  delay(10);
}

// ═══════════════════════════════════════════════════════════════
//  BUZZER (non-blocking, no tone() — uses simple digital pulses)
// ═══════════════════════════════════════════════════════════════

static uint8_t  buzzerStep   = 0;   // 0 = idle
static uint32_t buzzerStart  = 0;
static bool     startupDone  = false; // suppress beep on first scan

void buzzerConnect() {
  if (!startupDone) return;
  if (buzzerStep == 0) {
    buzzerStep  = 1;
    buzzerStart = millis();
    digitalWrite(BUZZER_PIN, HIGH);
  }
}

void buzzerUpdate() {
  if (buzzerStep == 0) return;
  uint32_t elapsed = millis() - buzzerStart;

  switch (buzzerStep) {
    case 1:
      if (elapsed >= 100) {
        digitalWrite(BUZZER_PIN, LOW);   // gap
        buzzerStep = 2;
      }
      break;
    case 2:
      if (elapsed >= 150) {
        digitalWrite(BUZZER_PIN, HIGH);  // second beep
        buzzerStep = 3;
      }
      break;
    case 3:
      if (elapsed >= 250) {
        digitalWrite(BUZZER_PIN, LOW);   // done
        buzzerStep = 0;
      }
      break;
  }
}

// ═══════════════════════════════════════════════════════════════
//  SENSOR INIT
// ═══════════════════════════════════════════════════════════════

bool initAHT() {
  if (!aht.begin(&Wire, 0, ADDR_AHT20)) return false;
  delay(20);  // let sensor stabilize after init
  // Do one throwaway measurement to prime the sensor
  sensors_event_t h, t;
  aht.getEvent(&h, &t);
  delay(80);  // AHT20 measurement takes ~75ms
  return true;
}

bool initBMP() {
  // Combo module BMP280 at 0x77
  return bmp.begin(ADDR_BMP280);
}

bool initMPU() {
  uint8_t id = i2cReadReg(ADDR_MPU6500, MPU_WHO_AM_I);
  // MPU6500=0x70, MPU6050=0x68
  if (id != 0x70 && id != 0x68) return false;
  i2cWriteReg(ADDR_MPU6500, MPU_PWR_MGMT_1, 0x00);  // wake up
  delay(10);
  i2cWriteReg(ADDR_MPU6500, MPU_ACCEL_CONFIG, 0x08); // ±4g
  i2cWriteReg(ADDR_MPU6500, MPU_GYRO_CONFIG,  0x08); // ±500°/s
  return true;
}

bool initQMC() {
  i2cWriteReg(ADDR_QMC5883, QMC_SET_RESET, 0x01);
  delay(5);
  // Continuous mode, 200 Hz ODR, 8 Gauss range, 512× oversampling
  i2cWriteReg(ADDR_QMC5883, QMC_CTRL1, 0x1D);
  delay(10);
  return true;
}

bool initGPS() {
  // u-blox M10 responds on I2C (DDC) — just verify presence
  return i2cProbe(ADDR_GPS);
}

bool initOLED() {
  // Software reset via I2C command before init
  Wire.beginTransmission(ADDR_OLED);
  Wire.write(0x00);   // command mode
  Wire.write(0xAE);   // display OFF
  Wire.endTransmission();
  delay(10);

  if (!oled.begin(ADDR_OLED, true)) return false;
  delay(50);

  // Clear any garbage in display RAM
  oled.clearDisplay();
  oled.display();
  delay(10);

  // Splash screen
  oled.clearDisplay();
  oled.setTextColor(SH110X_WHITE);
  oled.setTextSize(2);
  int16_t x = (128 - 6 * 12) / 2;
  oled.setCursor(x > 0 ? x : 0, 10);
  oled.print(F("NazarX"));
  oled.setTextSize(1);
  oled.setCursor(20, 40);
  oled.print(F("Hot-Plug Station"));
  oled.setCursor(28, 52);
  oled.print(F("Snap & Sense"));
  oled.display();
  delay(1200);
  return true;
}

// ═══════════════════════════════════════════════════════════════
//  SENSOR READ
// ═══════════════════════════════════════════════════════════════

void readAHT() {
  sensors_event_t h, t;
  aht.getEvent(&h, &t);
  ahtTemp  = t.temperature;
  ahtHumid = h.relative_humidity;
}

void readBMP() {
  bmpTemp  = bmp.readTemperature();
  bmpPress = bmp.readPressure() / 100.0f;  // Pa → hPa
  bmpAlt   = bmp.readAltitude(1013.25);
}

void readMPU() {
  Wire.beginTransmission(ADDR_MPU6500);
  Wire.write(MPU_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDR_MPU6500, (uint8_t)14);

  if (Wire.available() < 14) return;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // skip temp
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  // Accel → m/s² (±4g → 8192 LSB/g)
  float agx = ax / 8192.0f, agy = ay / 8192.0f, agz = az / 8192.0f;
  accX = agx * 9.81f;
  accY = agy * 9.81f;
  accZ = agz * 9.81f;

  // Gyro → deg/s (±500°/s → 65.5 LSB/°/s)
  gyrX = gx / 65.5f;
  gyrY = gy / 65.5f;
  gyrZ = gz / 65.5f;

  // Orientation from accel
  roll  = atan2(agy, agz);
  pitch = atan2(-agx, sqrt(agy * agy + agz * agz));
  // Yaw from gyro integration (no magnetometer fusion here)
  yaw  += gyrZ * 0.0174533f * (TELEM_INTERVAL / 1000.0f);
  while (yaw >  PI) yaw -= 2 * PI;
  while (yaw < -PI) yaw += 2 * PI;
}

void readQMC() {
  Wire.beginTransmission(ADDR_QMC5883);
  Wire.write(QMC_DATA_OUT);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDR_QMC5883, (uint8_t)6);

  if (Wire.available() < 6) return;

  magRaw[0] = Wire.read() | (Wire.read() << 8);  // X (little-endian)
  magRaw[1] = Wire.read() | (Wire.read() << 8);  // Y
  magRaw[2] = Wire.read() | (Wire.read() << 8);  // Z

  magHeading = atan2((float)magRaw[1], (float)magRaw[0]) * 57.2958f;
  if (magHeading < 0) magHeading += 360.0f;
}

// ── GPS: NMEA over I2C (u-blox DDC) ────────────────────────

float nmeaToDecimal(const char* raw, char dir) {
  float val = atof(raw);
  int deg   = (int)(val / 100);
  float min = val - deg * 100;
  float dec = deg + min / 60.0f;
  if (dir == 'S' || dir == 'W') dec = -dec;
  return dec;
}

void parseGGA(char* s) {
  // Split on commas — modifies buffer in place
  char* f[15];
  int n = 0;
  f[0] = s;
  for (char* p = s; *p && n < 14; p++) {
    if (*p == ',') {
      *p = '\0';
      f[++n] = p + 1;
    }
  }
  if (n < 9) return;

  gpsFix  = atoi(f[6]);
  gpsSats = atoi(f[7]);
  if (gpsFix > 0 && strlen(f[2]) > 0) {
    gpsLat = nmeaToDecimal(f[2], f[3][0]);
    gpsLon = nmeaToDecimal(f[4], f[5][0]);
    gpsAlt = atof(f[9]);
  }
}

void readGPS() {
  // Check how many bytes are waiting in the DDC buffer
  Wire.beginTransmission(ADDR_GPS);
  Wire.write(0xFD);  // bytes-available register (high byte)
  Wire.endTransmission(false);
  Wire.requestFrom(ADDR_GPS, (uint8_t)2);

  if (Wire.available() < 2) return;
  uint16_t avail = ((uint16_t)Wire.read() << 8) | Wire.read();
  if (avail == 0 || avail >= 0x7FFF) return;

  // Read ALL available bytes in 32-byte chunks (STM32 Wire buffer limit)
  while (avail > 0) {
    uint8_t toRead = (avail > 32) ? 32 : (uint8_t)avail;
    Wire.requestFrom(ADDR_GPS, toRead);
    uint8_t got = Wire.available();
    if (got == 0) break;   // bus error — stop
    avail -= got;

    while (Wire.available()) {
      char c = Wire.read();
      if (c == 0xFF) continue;  // DDC padding byte

      if (c == '$') {
        nmeaIdx    = 0;
        nmeaActive = true;
      }
      if (nmeaActive && nmeaIdx < sizeof(nmeaBuf) - 1) {
        nmeaBuf[nmeaIdx++] = c;
        if (c == '\n' || c == '\r') {
          nmeaBuf[nmeaIdx] = '\0';
          nmeaActive = false;
          if (strstr(nmeaBuf, "GGA")) {
            char* body = strchr(nmeaBuf, ',');
            if (body) parseGGA(body + 1);
          }
        }
      }
    }
  }
}

// ═══════════════════════════════════════════════════════════════
//  OLED UI
// ═══════════════════════════════════════════════════════════════

// ── Header bar (always visible, top 14 px) ──────────────────
void drawHeader() {
  oled.setTextSize(1);
  oled.setTextColor(SH110X_WHITE);
  oled.setCursor(2, 3);
  oled.print(F("NazarX"));

  // 5 indicator dots for sensor modules (AHT BMP MPU GPS MAG)
  const uint8_t ids[] = {M_AHT, M_BMP, M_MPU, M_GPS, M_QMC};
  for (int i = 0; i < 5; i++) {
    int x = 88 + i * 8;
    if (mod[ids[i]].connected)
      oled.fillCircle(x, 5, 3, SH110X_WHITE);
    else
      oled.drawCircle(x, 5, 3, SH110X_WHITE);
  }

  // Separator line
  oled.drawFastHLine(0, 13, 128, SH110X_WHITE);
}

// ── Card frame with inverted title bar ──────────────────────
void drawCard(const char* title) {
  // Outer rounded rectangle
  oled.drawRoundRect(0, 15, 128, 49, 4, SH110X_WHITE);

  // Inverted (white) title bar
  oled.fillRoundRect(2, 17, 124, 11, 2, SH110X_WHITE);
  oled.setTextColor(SH110X_BLACK);
  int tw = strlen(title) * 6;
  oled.setCursor((128 - tw) / 2, 19);
  oled.print(title);
  oled.setTextColor(SH110X_WHITE);
}

// ── Right-aligned value helper ──────────────────────────────
void drawRow(uint8_t row, const char* label, float val, uint8_t dec, const char* unit) {
  int y = 31 + row * 10;
  oled.setCursor(6, y);
  oled.print(label);

  char vbuf[16];
  dtostrf(val, 0, dec, vbuf);
  strcat(vbuf, unit);
  int vw = strlen(vbuf) * 6;
  oled.setCursor(122 - vw, y);
  oled.print(vbuf);
}

void drawRowStr(uint8_t row, const char* label, const char* val) {
  int y = 31 + row * 10;
  oled.setCursor(6, y);
  oled.print(label);
  int vw = strlen(val) * 6;
  oled.setCursor(122 - vw, y);
  oled.print(val);
}

// ── Active indicator at bottom-right of card ────────────────
void drawActive() {
  oled.fillCircle(100, 57, 2, SH110X_WHITE);
  oled.setCursor(106, 55);
  oled.print(F("OK"));
}

// ── Per-module pages ────────────────────────────────────────

void pageAHT() {
  drawCard("AHT20");
  drawRow(0, "Temp",     ahtTemp,  1, " C");
  drawRow(1, "Humidity", ahtHumid, 1, " %");
  drawActive();
}

void pageBMP() {
  drawCard("BMP280");
  drawRow(0, "Press", bmpPress, 1, " hPa");
  drawRow(1, "Alt",   bmpAlt,   1, " m");
  drawRow(2, "Temp",  bmpTemp,  1, " C");
}

void pageMPU() {
  drawCard("MPU6500");
  drawRow(0, "Roll",  roll  * 57.2958f, 1, " deg");
  drawRow(1, "Pitch", pitch * 57.2958f, 1, " deg");
  drawRow(2, "Yaw",   yaw   * 57.2958f, 1, " deg");
}

void pageGPS() {
  drawCard("GPS M10");

  char latBuf[14], lonBuf[14];
  dtostrf(gpsLat, 0, 5, latBuf);
  dtostrf(gpsLon, 0, 5, lonBuf);
  drawRowStr(0, "Lat", latBuf);
  drawRowStr(1, "Lon", lonBuf);

  char satBuf[12];
  snprintf(satBuf, sizeof(satBuf), "%dsat %s",
           gpsSats, gpsFix > 0 ? "3D" : "--");
  drawRowStr(2, "Fix", satBuf);
}

void pageQMC() {
  drawCard("COMPASS");
  drawRow(0, "Heading", magHeading, 1, " deg");

  char xyzBuf[20];
  snprintf(xyzBuf, sizeof(xyzBuf), "%d/%d/%d", magRaw[0], magRaw[1], magRaw[2]);
  drawRowStr(1, "XYZ", xyzBuf);
  drawActive();
}

void pageEmpty() {
  oled.drawRoundRect(0, 15, 128, 49, 4, SH110X_WHITE);
  oled.setCursor(8, 28);
  oled.print(F("No sensors found"));
  oled.setCursor(8, 42);
  oled.print(F("Snap to connect!"));
}

// ── Render current page ─────────────────────────────────────
void updateDisplay() {
  if (!mod[M_OLED].connected || !mod[M_OLED].inited) return;

  oled.clearDisplay();
  drawHeader();

  if (dispCount == 0) {
    pageEmpty();
  } else {
    uint8_t m = dispList[dispPage % dispCount];
    switch (m) {
      case M_AHT: pageAHT(); break;
      case M_BMP: pageBMP(); break;
      case M_MPU: pageMPU(); break;
      case M_GPS: pageGPS(); break;
      case M_QMC: pageQMC(); break;
    }
  }

  oled.display();
}

// ═══════════════════════════════════════════════════════════════
//  HOT-PLUG I2C SCANNER
// ═══════════════════════════════════════════════════════════════

void rebuildDisplayList() {
  dispCount = 0;
  for (uint8_t i = 0; i < MOD_N; i++) {
    if (i == M_OLED) continue;  // OLED is the display, not a data page
    if (mod[i].connected) {
      dispList[dispCount++] = i;
    }
  }
  if (dispPage >= dispCount && dispCount > 0) {
    dispPage = 0;
  }
}

void probeAll(bool* detected) {
  detected[M_AHT]  = i2cProbe(ADDR_AHT20);
  detected[M_BMP]  = i2cProbe(ADDR_BMP280);
  detected[M_MPU]  = i2cProbe(ADDR_MPU6500);
  detected[M_GPS]  = i2cProbe(ADDR_GPS);
  detected[M_QMC]  = i2cProbe(ADDR_QMC5883);
  detected[M_OLED] = i2cProbe(ADDR_OLED);
}

void scanI2C() {
  bool detected[MOD_N];
  probeAll(detected);

  // If nothing found, bus may be stuck — recover and retry once
  bool anyFound = false;
  for (uint8_t i = 0; i < MOD_N; i++) {
    if (detected[i]) { anyFound = true; break; }
  }
  if (!anyFound) {
    i2cRecover();
    probeAll(detected);
  }

  bool changed = false;
  bool newConnection = false;   // beep once at end, not per module

  for (uint8_t i = 0; i < MOD_N; i++) {
    mod[i].wasConnected = mod[i].connected;

    if (detected[i]) {
      // ── Probe succeeded ──
      mod[i].missCount = 0;

      if (!mod[i].wasConnected) {
        // NEW CONNECTION
        mod[i].connected = true;
        Serial.print(F("[+] "));
        Serial.println(MOD_NAMES[i]);

        bool ok = false;
        switch (i) {
          case M_AHT:  ok = initAHT();  break;
          case M_BMP:  ok = initBMP();   break;
          case M_MPU:  ok = initMPU();   break;
          case M_GPS:  ok = initGPS();   break;
          case M_QMC:  ok = initQMC();   break;
          case M_OLED: ok = initOLED();  break;
        }
        mod[i].inited = ok;
        Serial.println(ok ? F("    init OK") : F("    init FAIL"));
        if (ok) newConnection = true;
        changed = true;
      }
    } else {
      // ── Probe failed ──
      if (mod[i].connected) {
        mod[i].missCount++;
        if (mod[i].missCount >= DISCONNECT_THRESHOLD) {
          // Confirmed disconnection
          mod[i].connected = false;
          mod[i].inited    = false;
          mod[i].missCount = 0;
          Serial.print(F("[-] "));
          Serial.println(MOD_NAMES[i]);
          changed = true;
        }
      }
    }
  }

  // Single beep for the whole batch of new connections
  if (newConnection) buzzerConnect();

  if (changed) {
    rebuildDisplayList();
  }
}

// ═══════════════════════════════════════════════════════════════
//  READ ALL CONNECTED SENSORS
// ═══════════════════════════════════════════════════════════════

void readAllSensors() {
  if (mod[M_AHT].connected && mod[M_AHT].inited) readAHT();
  if (mod[M_BMP].connected && mod[M_BMP].inited) readBMP();
  if (mod[M_MPU].connected && mod[M_MPU].inited) readMPU();
  if (mod[M_GPS].connected && mod[M_GPS].inited) readGPS();
  if (mod[M_QMC].connected && mod[M_QMC].inited) readQMC();
}

// ═══════════════════════════════════════════════════════════════
//  MAVLINK TELEMETRY
// ═══════════════════════════════════════════════════════════════

void sendTelemetry() {
  if (!telemetry_enabled) return;

  mavlink_message_t msg;
  uint8_t buf[64];
  uint16_t len;

  // 1) SCALED_PRESSURE (BMP280)
  if (mod[M_BMP].connected) {
    mavlink_msg_scaled_pressure_pack(
      1, MAV_COMP_ID_AUTOPILOT1, &msg,
      millis(), bmpPress, 0,
      (int16_t)(bmpTemp * 100), 0
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    lora.write(buf, len);
  }

  // 2) VFR_HUD — altitude
  if (mod[M_BMP].connected) {
    mavlink_msg_vfr_hud_pack(
      1, MAV_COMP_ID_AUTOPILOT1, &msg,
      0, 0, 0, 0, bmpAlt, 0
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    lora.write(buf, len);
  }

  // 3) HUMIDITY (AHT20)
  if (mod[M_AHT].connected) {
    mavlink_msg_named_value_float_pack(
      1, MAV_COMP_ID_AUTOPILOT1, &msg,
      millis(), "HUMIDITY", ahtHumid
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    lora.write(buf, len);
  }

  // 4) SCALED_IMU2 (MPU6500 + QMC5883L)
  mavlink_msg_scaled_imu2_pack(
    1, MAV_COMP_ID_AUTOPILOT1, &msg,
    millis(),
    (int16_t)(accX * 100), (int16_t)(accY * 100), (int16_t)(accZ * 100),
    (int16_t)(gyrX * 17.4533f), (int16_t)(gyrY * 17.4533f), (int16_t)(gyrZ * 17.4533f),
    magRaw[0], magRaw[1], magRaw[2],
    0
  );
  len = mavlink_msg_to_send_buffer(buf, &msg);
  lora.write(buf, len);

  // 5) ATTITUDE (MPU6500)
  mavlink_msg_attitude_pack(
    1, MAV_COMP_ID_AUTOPILOT1, &msg,
    millis(), roll, pitch, yaw, 0, 0, 0
  );
  len = mavlink_msg_to_send_buffer(buf, &msg);
  lora.write(buf, len);

  // 6) GPS_RAW_INT
  if (mod[M_GPS].connected && gpsFix > 0) {
    mavlink_msg_gps_raw_int_pack(
      1, MAV_COMP_ID_AUTOPILOT1, &msg,
      (uint64_t)millis() * 1000,
      gpsFix,
      (int32_t)(gpsLat * 1e7),
      (int32_t)(gpsLon * 1e7),
      (int32_t)(gpsAlt * 1000),
      0xFFFF, 0xFFFF,        // eph, epv (unknown)
      0xFFFF,                // vel (unknown)
      0xFFFF,                // cog (unknown)
      gpsSats,
      0, 0, 0, 0, 0, 0       // alt_ellipsoid, h_acc, v_acc, vel_acc, hdg_acc, yaw
    );
    len = mavlink_msg_to_send_buffer(buf, &msg);
    lora.write(buf, len);
  }

  // 7) HEARTBEAT with module bitmask in custom_mode
  // Each bit = one connected module (AHT=b0, BMP=b1, MPU=b2, GPS=b3, QMC=b4, OLED=b5)
  uint32_t modBits = 0;
  for (uint8_t i = 0; i < MOD_N; i++) {
    if (mod[i].connected) modBits |= (1 << i);
  }
  mavlink_msg_heartbeat_pack(
    1, MAV_COMP_ID_AUTOPILOT1, &msg,
    MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC,
    MAV_MODE_FLAG_SAFETY_ARMED, modBits, MAV_STATE_ACTIVE
  );
  len = mavlink_msg_to_send_buffer(buf, &msg);
  lora.write(buf, len);
}

// ═══════════════════════════════════════════════════════════════
//  INCOMING COMMANDS FROM GROUND STATION
// ═══════════════════════════════════════════════════════════════

void checkIncomingCommands() {
  static mavlink_message_t rx_msg;
  static mavlink_status_t  rx_status;

  while (lora.available()) {
    uint8_t c = lora.read();
    if (mavlink_parse_char(MAVLINK_COMM_1, c, &rx_msg, &rx_status)) {
      if (rx_msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
        mavlink_statustext_t st;
        mavlink_msg_statustext_decode(&rx_msg, &st);

        String cmd = String(st.text);
        cmd.trim();
        String prefix = String("CMD,") + TEAM_ID + ",";
        if (!cmd.startsWith(prefix)) continue;

        String action = cmd.substring(prefix.length());
        if (action == "ON")       telemetry_enabled = true;
        else if (action == "OFF") telemetry_enabled = false;
        else if (action == "CAL") { roll = pitch = yaw = 0; }
      }
    }
  }
}

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(9600);
  delay(2000);

  Serial.println(F("═══════════════════════════════════"));
  Serial.println(F(" ActInSpace - Hot-Plug I2C Station"));
  Serial.println(F("═══════════════════════════════════"));

  // Buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);

  // I2C bus
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  Wire.setClock(100000);
  Wire.setTimeout(25);     // 25 ms timeout — prevents hang on hot-unplug
  delay(200);

  // I2C bus scan — show all detected addresses
  Serial.println(F("I2C scan:"));
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  0x"));
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
    }
  }
  Serial.println();

  // Clear module state
  for (uint8_t i = 0; i < MOD_N; i++) {
    mod[i].connected    = false;
    mod[i].wasConnected = false;
    mod[i].inited       = false;
    mod[i].missCount    = 0;
  }

  // LoRa UART
  lora.begin(57600);
  delay(100);
  while (lora.available()) lora.read();

  // No startup beep — buzzer only sounds on new module detection

  // Initial scan (no beep — startupDone is still false)
  scanI2C();
  startupDone = true;   // future connections will beep

  Serial.println(F("Ready — snap magnetic connectors to attach sensors"));
  Serial.println();
}

// ═══════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════

void loop() {
  uint32_t now = millis();

  // ── Drive non-blocking buzzer ──
  buzzerUpdate();

  // ── Check ground station commands ──
  checkIncomingCommands();

  // ── Periodic I2C scan for hot-plug detection ──
  if (now - lastScan >= SCAN_INTERVAL) {
    lastScan = now;
    scanI2C();
  }

  // ── Read sensors & send telemetry at 10 Hz ──
  if (now - lastTelem >= TELEM_INTERVAL) {
    lastTelem = now;
    counter++;

    readAllSensors();
    sendTelemetry();

    // Debug output every 10th cycle
    if (counter % 10 == 0) {
      Serial.print(F("#"));
      Serial.print(counter);
      Serial.print(F(" ["));
      for (uint8_t i = 0; i < MOD_N; i++) {
        Serial.print(mod[i].connected ? '+' : '-');
      }
      Serial.print(F("]"));
      if (mod[M_AHT].connected) {
        Serial.print(F(" T:"));  Serial.print(ahtTemp, 1);
        Serial.print(F("C H:")); Serial.print(ahtHumid, 1);
        Serial.print(F("%"));
      }
      if (mod[M_BMP].connected) {
        Serial.print(F(" P:"));   Serial.print(bmpPress, 0);
        Serial.print(F("hPa A:")); Serial.print(bmpAlt, 1);
        Serial.print(F("m"));
      }
      if (mod[M_GPS].connected) {
        // Show raw DDC byte count to verify I2C data flow
        Wire.beginTransmission(ADDR_GPS);
        Wire.write(0xFD);
        Wire.endTransmission(false);
        Wire.requestFrom(ADDR_GPS, (uint8_t)2);
        uint16_t gpsAvail = 0;
        if (Wire.available() >= 2) {
          gpsAvail = ((uint16_t)Wire.read() << 8) | Wire.read();
          if (gpsAvail >= 0x7FFF) gpsAvail = 0;
        }
        Serial.print(F(" DDC:")); Serial.print(gpsAvail);
        Serial.print(F("B Sat:")); Serial.print(gpsSats);
        Serial.print(F(" Fix:")); Serial.print(gpsFix);
        Serial.print(F(" Lat:")); Serial.print(gpsLat, 6);
        Serial.print(F(" Lon:")); Serial.print(gpsLon, 6);
        Serial.print(F(" Alt:")); Serial.print(gpsAlt, 1);
      }
      Serial.println();
    }
  }

  // ── Cycle OLED display page every 2 seconds ──
  if (now - lastDisp >= DISPLAY_INTERVAL) {
    lastDisp = now;
    if (dispCount > 0) {
      dispPage = (dispPage + 1) % dispCount;
    }
    updateDisplay();
  }
}
