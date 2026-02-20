/*
 * MAVLink Transmitter with BMP280 for LR900 LoRa
 * Sends heartbeat and pressure data (ArduPilot compatible)
 *
 * Connections:
 * - LR900 TX → Arduino D2
 * - LR900 RX → Arduino D3
 * - LR900 VCC → Arduino 5V
 * - LR900 GND → Arduino GND
 * - BMP280: SDA (A4), SCL (A5) - I2C
 */

#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>
#include <MAVLink.h>

// LR900 connected to D2 and D3
SoftwareSerial lora(2, 3); // RX=D2, TX=D3

// BMP280 sensor
Adafruit_BMP280 bmp;

uint32_t counter = 0;
unsigned long previousMillis = 0;
const long interval = 100; // Send every 100ms (10Hz - SpeedyBee rate)

void setup() {
  // Debug serial (to computer)
  Serial.begin(9600);
  Serial.println("MAVLink Transmitter Starting...");

  // Initialize I2C
  Wire.begin();

  // Initialize BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 not found at 0x76, trying 0x77...");
    if (!bmp.begin(0x77)) {
      Serial.println("Could not find BMP280!");
      while (1) delay(10);
    }
  }
  Serial.println("BMP280 found!");

  // Configure BMP280
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  // LoRa serial
  lora.begin(57600);
  delay(1000);

  Serial.println("Setup complete! Sending MAVLink at 10Hz (100ms interval).");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    counter++;

    // Read BMP280 sensor
    float temperature = bmp.readTemperature();
    float pressure = bmp.readPressure() / 100.0F; // Convert to hPa
    float altitude = bmp.readAltitude(1013.25);   // Calculate altitude (sea level pressure)

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // 1. Send SCALED_PRESSURE (same as ArduPilot)
    mavlink_msg_scaled_pressure_pack(
      1,                                        // System ID
      MAV_COMP_ID_AUTOPILOT1,                  // Component ID
      &msg,
      millis(),                                 // time_boot_ms
      pressure,                                 // press_abs (hPa)
      0,                                        // press_diff (differential pressure, not used)
      (int16_t)(temperature * 100),            // temperature (centi-degrees C)
      0                                         // temperature_press_diff (not used)
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    lora.write(buf, len);

    // 2. Send VFR_HUD with altitude (ArduPilot format)
    mavlink_msg_vfr_hud_pack(
      1,                                        // System ID
      MAV_COMP_ID_AUTOPILOT1,                  // Component ID
      &msg,
      0,                                        // airspeed (m/s) - not used
      0,                                        // groundspeed (m/s) - not used
      0,                                        // heading (degrees) - not used
      0,                                        // throttle (%) - not used
      altitude,                                 // altitude (MSL in meters)
      0                                         // climb rate (m/s) - not used
    );

    len = mavlink_msg_to_send_buffer(buf, &msg);
    lora.write(buf, len);

    // 3. Send heartbeat with counter
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

    // Debug output
    Serial.print("#");
    Serial.print(counter);
    Serial.print(" T:");
    Serial.print(temperature, 1);
    Serial.print("C P:");
    Serial.print(pressure, 1);
    Serial.print("hPa Alt:");
    Serial.print(altitude, 1);
    Serial.println("m");
  }
}