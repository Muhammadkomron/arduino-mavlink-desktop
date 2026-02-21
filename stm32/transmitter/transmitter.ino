/*
 * STM32F401 MAVLink Transmitter with BMP280 + AHT20 + MPU9250/6500/6050
 * Sends telemetry data at 10Hz via LoRa
 *
 * Hardware:
 * - STM32F401CCU6 (WeAct Black Pill) or similar
 * - LoRa module (e.g., LR900, SX1278)
 * - BMP280 pressure sensor
 * - AHT20 humidity sensor
 * - MPU9250/6500/6050 IMU (auto-detected)
 *   - MPU9250: 9-axis (accel + gyro + magnetometer)
 *   - MPU6500: 6-axis (accel + gyro)
 *   - MPU6050: 6-axis (accel + gyro)
 *
 * Connections:
 * LoRa (PA9/PA10 - Most reliable):
 * - LoRa TX → STM32 PA10 (RX)
 * - LoRa RX → STM32 PA9 (TX)
 * - LoRa VCC → 3.3V
 * - LoRa GND → GND
 *
 * I2C Sensors (I2C1):
 * - BMP280: SDA → PB7, SCL → PB6
 * - AHT20: SDA → PB7, SCL → PB6 (shared)
 * - MPU9250/6500/6050: SDA → PB7, SCL → PB6 (shared)
 * - All sensors: VCC → 3.3V, GND → GND
 *
 * Alternative I2C (I2C2):
 * - SDA → PB11, SCL → PB10
 *
 * Power: Add 100-470µF capacitor between VCC and GND for stability
 */

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>
#include <MAVLink.h>

// MPU9250/6500/6050 I2C registers
#define MPU_ADDR 0x68
#define MPU_WHO_AM_I 0x75
#define MPU_PWR_MGMT_1 0x6B
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_TEMP_OUT_H 0x41
#define MPU_GYRO_XOUT_H 0x43
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_GYRO_CONFIG 0x1B
#define MPU_INT_PIN_CFG 0x37
#define MPU_USER_CTRL 0x6A

// AK8963 magnetometer (inside MPU9250)
#define AK8963_ADDR 0x0C
#define AK8963_WHO_AM_I 0x00
#define AK8963_ST1 0x02
#define AK8963_XOUT_L 0x03
#define AK8963_CNTL 0x0A

// LoRa serial on PA9/PA10 (USART1)
HardwareSerial lora(PA10, PA9); // RX=PA10, TX=PA9

// Sensors
Adafruit_BMP280 bmp;  // I2C
Adafruit_AHTX0 aht;   // I2C
// MPU9250/6500/6050 using raw I2C (no library needed)

// Sensor status flags
bool bmp_ok = false;
bool aht_ok = false;
bool mpu_ok = false;
bool mag_ok = false;  // Magnetometer (only on MPU9250)
uint8_t mpu_chip_id = 0;  // Store chip ID

uint32_t counter = 0;
unsigned long previousMillis = 0;
const long interval = 100; // Send every 100ms (10Hz - ArduPilot rate)

// MPU9250/6500/6050 helper functions
void mpu_write_reg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t mpu_read_reg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1);
  return Wire.read();
}

void mpu_read_raw(int16_t *accel, int16_t *gyro, int16_t *temp) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);

  if (Wire.available() >= 14) {
    accel[0] = (Wire.read() << 8) | Wire.read(); // X
    accel[1] = (Wire.read() << 8) | Wire.read(); // Y
    accel[2] = (Wire.read() << 8) | Wire.read(); // Z
    *temp = (Wire.read() << 8) | Wire.read();    // Temperature
    gyro[0] = (Wire.read() << 8) | Wire.read();  // X
    gyro[1] = (Wire.read() << 8) | Wire.read();  // Y
    gyro[2] = (Wire.read() << 8) | Wire.read();  // Z
  }
}

// AK8963 magnetometer helper (inside MPU9250)
bool mag_read_raw(int16_t *mag) {
  // Check if data is ready
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(AK8963_ST1);
  Wire.endTransmission(false);
  Wire.requestFrom(AK8963_ADDR, 1);

  if (Wire.read() & 0x01) {  // Data ready bit
    // Read magnetometer data
    Wire.beginTransmission(AK8963_ADDR);
    Wire.write(AK8963_XOUT_L);
    Wire.endTransmission(false);
    Wire.requestFrom(AK8963_ADDR, 7);

    if (Wire.available() >= 7) {
      mag[0] = Wire.read() | (Wire.read() << 8);  // X (little-endian)
      mag[1] = Wire.read() | (Wire.read() << 8);  // Y
      mag[2] = Wire.read() | (Wire.read() << 8);  // Z
      Wire.read();  // ST2 status register (must read to trigger next measurement)
      return true;
    }
  }
  return false;
}

// Orientation angles
float roll = 0, pitch = 0, yaw = 0;

void setup() {
  // Debug serial (USB)
  Serial.begin(9600);

  // Wait for power stabilization
  delay(2000);

  Serial.println(F("STM32 MAVLink Transmitter with BMP280 + AHT20 + MPU9250/6500/6050"));
  Serial.println(F("Starting initialization..."));

  // Initialize I2C (PB7=SDA, PB6=SCL for I2C1)
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  Wire.setClock(100000);  // 100kHz - slower speed for stability
  delay(200);

  // Initialize MPU9250/6500/6050 with raw I2C (most reliable method)
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() == 0) {
    // Check WHO_AM_I register
    mpu_chip_id = mpu_read_reg(MPU_WHO_AM_I);

    if (mpu_chip_id == 0x68 || mpu_chip_id == 0x70 || mpu_chip_id == 0x71 || mpu_chip_id == 0x73) {
      // Wake up MPU (disable sleep)
      mpu_write_reg(MPU_PWR_MGMT_1, 0x00);
      delay(50);

      // Configure accelerometer (±4g)
      mpu_write_reg(MPU_ACCEL_CONFIG, 0x08);

      // Configure gyroscope (±500°/s)
      mpu_write_reg(MPU_GYRO_CONFIG, 0x08);

      mpu_ok = true;

      // Print chip name
      Serial.print(F("MPU"));
      if (mpu_chip_id == 0x71) {
        Serial.println(F("9250 found and configured! (9-axis)"));
      } else if (mpu_chip_id == 0x70) {
        Serial.println(F("6500 found and configured! (6-axis)"));
      } else if (mpu_chip_id == 0x73) {
        Serial.println(F("9255 found and configured! (9-axis)"));
      } else {
        Serial.println(F("6050 found and configured! (6-axis)"));
      }

      // Try to initialize magnetometer (MPU9250/9255 only)
      if (mpu_chip_id == 0x71 || mpu_chip_id == 0x73) {
        // Enable I2C bypass to access magnetometer
        mpu_write_reg(MPU_INT_PIN_CFG, 0x02);
        delay(10);
        mpu_write_reg(MPU_USER_CTRL, 0x00);
        delay(10);

        // Check if AK8963 is present
        Wire.beginTransmission(AK8963_ADDR);
        if (Wire.endTransmission() == 0) {
          Wire.beginTransmission(AK8963_ADDR);
          Wire.write(AK8963_WHO_AM_I);
          Wire.endTransmission(false);
          Wire.requestFrom(AK8963_ADDR, 1);
          uint8_t mag_id = Wire.read();

          if (mag_id == 0x48) {  // AK8963 WHO_AM_I value
            // Set magnetometer to continuous measurement mode (100Hz)
            Wire.beginTransmission(AK8963_ADDR);
            Wire.write(AK8963_CNTL);
            Wire.write(0x16);  // 16-bit output, continuous mode 2 (100Hz)
            Wire.endTransmission();
            delay(10);

            mag_ok = true;
            Serial.println(F("  └─ AK8963 magnetometer enabled!"));
          }
        }
      }
    }
  }

  if (!mpu_ok) {
    Serial.println(F("WARNING: MPU not found"));
  }
  delay(200);

  // Initialize AHT20 (optional - continue even if not found)
  Serial.println(F("Initializing AHT20..."));
  delay(100);
  aht_ok = aht.begin();
  if (aht_ok) {
    Serial.println(F("AHT20 found!"));
  } else {
    Serial.println(F("WARNING: AHT20 not found - humidity will be 0"));
  }
  delay(200);

  // Initialize BMP280 (optional - continue even if not found)
  Serial.println(F("Initializing BMP280..."));
  delay(100);
  if (bmp.begin(0x76) || bmp.begin(0x77)) {
    bmp_ok = true;
    Serial.println(F("BMP280 found!"));
    // Configure BMP280 for weather monitoring
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  } else {
    Serial.println(F("WARNING: BMP280 not found - temp/pressure will be 0"));
  }
  delay(200);

  // Initialize LoRa serial
  lora.begin(57600);

  // Clear any garbage in buffer
  delay(100);
  while(lora.available()) {
    lora.read();
  }

  delay(1000);

  Serial.println(F("Setup complete! Sending MAVLink at 10Hz."));
  Serial.print(F("Sensors: BMP280 + AHT20 + MPU"));
  if (mpu_chip_id == 0x71) Serial.print(F("9250"));
  else if (mpu_chip_id == 0x70) Serial.print(F("6500"));
  else if (mpu_chip_id == 0x73) Serial.print(F("9255"));
  else Serial.print(F("6050"));
  Serial.print(F(" ("));
  Serial.print(mag_ok ? "9" : "6");
  Serial.println(F("-axis)"));
  delay(2000);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    counter++;

    // Read AHT20 sensor (or use defaults if not present)
    float humidity = 0;
    if (aht_ok) {
      sensors_event_t humidity_event, temp_event;
      aht.getEvent(&humidity_event, &temp_event);
      humidity = humidity_event.relative_humidity;
    }

    // Read BMP280 sensor (or use defaults if not present)
    float temperature = 0;
    float pressure = 1013.25;
    float altitude = 0;
    if (bmp_ok) {
      temperature = bmp.readTemperature();
      pressure = bmp.readPressure() / 100.0F; // Convert Pa to hPa
      altitude = bmp.readAltitude(1013.25);   // Sea level pressure
    }

    // Read MPU9250/6500/6050 using raw I2C (or use defaults if not present)
    float accel_x = 0, accel_y = 0, accel_z = 0;
    float gyro_x = 0, gyro_y = 0, gyro_z = 0;
    float mag_x = 0, mag_y = 0, mag_z = 0;
    float mpu_temp = 0;

    if (mpu_ok) {
      int16_t raw_accel[3], raw_gyro[3], raw_temp;
      mpu_read_raw(raw_accel, raw_gyro, &raw_temp);

      // Convert temperature (Formula: Temp = raw/340 + 36.53)
      mpu_temp = raw_temp / 340.0 + 36.53;

      // Convert raw accelerometer to m/s² (±4g range, 8192 LSB/g)
      float accel_g_x = raw_accel[0] / 8192.0;
      float accel_g_y = raw_accel[1] / 8192.0;
      float accel_g_z = raw_accel[2] / 8192.0;
      accel_x = accel_g_x * 9.81;
      accel_y = accel_g_y * 9.81;
      accel_z = accel_g_z * 9.81;

      // Convert raw gyroscope to deg/s (±500°/s range, 65.5 LSB/°/s)
      gyro_x = raw_gyro[0] / 65.5;
      gyro_y = raw_gyro[1] / 65.5;
      gyro_z = raw_gyro[2] / 65.5;

      // Read magnetometer if available (MPU9250/9255 only)
      if (mag_ok) {
        int16_t raw_mag[3];
        if (mag_read_raw(raw_mag)) {
          // Convert to µT (sensitivity 0.15 µT/LSB in 16-bit mode)
          mag_x = raw_mag[0] * 0.15;
          mag_y = raw_mag[1] * 0.15;
          mag_z = raw_mag[2] * 0.15;

          // Calculate yaw from magnetometer (tilt-compensated)
          float mag_x_comp = mag_x * cos(pitch) + mag_z * sin(pitch);
          float mag_y_comp = mag_x * sin(roll) * sin(pitch) + mag_y * cos(roll) - mag_z * sin(roll) * cos(pitch);
          yaw = atan2(-mag_y_comp, mag_x_comp);
        }
      } else {
        // No magnetometer - integrate gyro Z for yaw
        yaw += gyro_z * 0.0174533 * (interval / 1000.0);

        // Normalize yaw to ±π to prevent overflow
        while (yaw > PI) yaw -= 2 * PI;
        while (yaw < -PI) yaw += 2 * PI;
      }

      // Calculate roll and pitch from accelerometer
      roll = atan2(accel_g_y, accel_g_z);
      pitch = atan2(-accel_g_x, sqrt(accel_g_y * accel_g_y + accel_g_z * accel_g_z));
    }

    // Send MAVLink messages
    mavlink_message_t msg;
    uint8_t buf[64];  // Smaller buffer to save RAM

    // 1. Send SCALED_PRESSURE (ArduPilot format)
    mavlink_msg_scaled_pressure_pack(
      1,                                        // System ID
      MAV_COMP_ID_AUTOPILOT1,                  // Component ID
      &msg,
      millis(),                                 // time_boot_ms
      pressure,                                 // press_abs (hPa)
      0,                                        // press_diff (not used)
      (int16_t)(temperature * 100),            // temperature (0.01°C)
      0                                         // temperature_press_diff (not used)
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    lora.write(buf, len);

    // 2. Send VFR_HUD with altitude
    mavlink_msg_vfr_hud_pack(
      1,                                        // System ID
      MAV_COMP_ID_AUTOPILOT1,                  // Component ID
      &msg,
      0,                                        // airspeed (not used)
      0,                                        // groundspeed (not used)
      0,                                        // heading (not used)
      0,                                        // throttle (not used)
      altitude,                                 // altitude (MSL in meters)
      0                                         // climb rate (not used)
    );

    len = mavlink_msg_to_send_buffer(buf, &msg);
    lora.write(buf, len);

    // 3. Send humidity as NAMED_VALUE_FLOAT
    mavlink_msg_named_value_float_pack(
      1,                                        // System ID
      MAV_COMP_ID_AUTOPILOT1,                  // Component ID
      &msg,
      millis(),                                 // time_boot_ms
      "HUMIDITY",                               // name (max 10 chars)
      humidity                                  // value
    );

    len = mavlink_msg_to_send_buffer(buf, &msg);
    lora.write(buf, len);

    // 4. Send IMU data (accelerometer + gyroscope + magnetometer) as SCALED_IMU2
    mavlink_msg_scaled_imu2_pack(
      1,                                        // System ID
      MAV_COMP_ID_AUTOPILOT1,                  // Component ID
      &msg,
      millis(),                                 // time_boot_ms
      (int16_t)(accel_x * 100),                // xacc (mG)
      (int16_t)(accel_y * 100),                // yacc (mG)
      (int16_t)(accel_z * 100),                // zacc (mG)
      (int16_t)(gyro_x * 17.4533),             // xgyro (millirad/s)
      (int16_t)(gyro_y * 17.4533),             // ygyro (millirad/s)
      (int16_t)(gyro_z * 17.4533),             // zgyro (millirad/s)
      (int16_t)(mag_x * 10),                   // xmag (0.1 mT)
      (int16_t)(mag_y * 10),                   // ymag (0.1 mT)
      (int16_t)(mag_z * 10),                   // zmag (0.1 mT)
      0                                         // temperature (not used)
    );

    len = mavlink_msg_to_send_buffer(buf, &msg);
    lora.write(buf, len);

    // 5. Send attitude (roll, pitch, yaw)
    mavlink_msg_attitude_pack(
      1,                                        // System ID
      MAV_COMP_ID_AUTOPILOT1,                  // Component ID
      &msg,
      millis(),                                 // time_boot_ms
      roll,                                     // roll (radians)
      pitch,                                    // pitch (radians)
      yaw,                                      // yaw (radians)
      0,                                        // rollspeed (rad/s) - not used
      0,                                        // pitchspeed (rad/s) - not used
      0                                         // yawspeed (rad/s) - not used
    );

    len = mavlink_msg_to_send_buffer(buf, &msg);
    lora.write(buf, len);

    // 6. Send heartbeat with counter
    mavlink_msg_heartbeat_pack(
      1,                              // System ID
      MAV_COMP_ID_AUTOPILOT1,        // Component ID
      &msg,
      MAV_TYPE_GENERIC,              // Type
      MAV_AUTOPILOT_GENERIC,         // Autopilot
      MAV_MODE_FLAG_SAFETY_ARMED,    // Mode
      counter,                        // Custom mode (our counter!)
      MAV_STATE_ACTIVE               // State
    );

    len = mavlink_msg_to_send_buffer(buf, &msg);
    lora.write(buf, len);

    // Compact debug output (every 10th message to reduce RAM usage)
    if (counter % 10 == 0) {
      Serial.print(F("#"));
      Serial.print(counter);
      Serial.print(F(" R:"));
      Serial.print(roll * 57.2958, 1);  // Convert to degrees
      Serial.print(F("° P:"));
      Serial.print(pitch * 57.2958, 1);
      Serial.print(F("° Y:"));
      Serial.print(yaw * 57.2958, 1);
      Serial.print(F("° | T:"));
      Serial.print(temperature, 1);
      Serial.print(F("C H:"));
      Serial.print(humidity, 1);
      Serial.print(F("% Alt:"));
      Serial.print(altitude, 1);
      Serial.println(F("m"));
    }
  }
}