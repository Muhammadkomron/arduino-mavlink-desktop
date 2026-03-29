package integration

import (
	"testing"

	"cansat-ground-station/backend"

	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
)

// TestFullTelemetryPipeline tests processing multiple MAVLink messages
// in sequence and verifying the aggregated telemetry state is correct.
func TestFullTelemetryPipeline(t *testing.T) {
	app := backend.NewApp()

	// 1. Process GPS data
	app.ProcessGpsRawInt(&common.MessageGpsRawInt{
		Lat:               413111000,
		Lon:               692797000,
		Alt:               450000,
		SatellitesVisible: 12,
		FixType:           3,
	})

	// 2. Process pressure + temperature
	app.ProcessScaledPressure(&common.MessageScaledPressure{
		PressAbs:    1013.25,
		Temperature: 2500,
	})

	// 3. Process altitude
	app.ProcessVfrHud(&common.MessageVfrHud{
		Alt: 150.5,
	})

	// 4. Process battery voltage
	app.ProcessSysStatus(&common.MessageSysStatus{
		VoltageBattery: 12500,
	})

	// 5. Process IMU
	app.ProcessScaledImu2(&common.MessageScaledImu2{
		Xacc:  980,
		Yacc:  0,
		Zacc:  -980,
		Xgyro: 1745,
		Ygyro: 0,
		Zgyro: 0,
	})

	// 6. Process attitude
	app.ProcessAttitude(&common.MessageAttitude{
		Roll:  0.5236,
		Pitch: -0.2618,
		Yaw:   1.5708,
	})

	// 7. Process humidity
	app.ProcessNamedValueFloat("HUMIDITY", 65.5)

	// Verify all telemetry is aggregated correctly
	td := app.GetTelemetry()

	if td.GpsLat != 41.3111 {
		t.Errorf("GPS lat: expected 41.3111, got %f", td.GpsLat)
	}
	if td.GpsLon != 69.2797 {
		t.Errorf("GPS lon: expected 69.2797, got %f", td.GpsLon)
	}
	if td.GpsAlt != 450.0 {
		t.Errorf("GPS alt: expected 450.0, got %f", td.GpsAlt)
	}
	if td.GpsSats != 12 {
		t.Errorf("GPS sats: expected 12, got %d", td.GpsSats)
	}
	if td.Pressure != 1013.25 {
		t.Errorf("pressure: expected 1013.25, got %f", td.Pressure)
	}
	if td.Temperature != 25.0 {
		t.Errorf("temperature: expected 25.0, got %f", td.Temperature)
	}
	if td.Altitude != 150.5 {
		t.Errorf("altitude: expected 150.5, got %f", td.Altitude)
	}
	if td.Voltage != 12.5 {
		t.Errorf("voltage: expected 12.5, got %f", td.Voltage)
	}
	if td.Humidity != 65.5 {
		t.Errorf("humidity: expected 65.5, got %f", td.Humidity)
	}

	expectedRoll := float32(0.5236) * 57.2958
	if td.Roll != expectedRoll {
		t.Errorf("roll: expected %f, got %f", expectedRoll, td.Roll)
	}
}

// TestMultipleHeartbeatsWithPacketLoss tests packet loss tracking
// across a realistic sequence of heartbeats.
func TestMultipleHeartbeatsWithPacketLoss(t *testing.T) {
	app := backend.NewApp()

	// Normal sequence: 1,2,3
	app.ProcessHeartbeat(1)
	app.ProcessHeartbeat(2)
	app.ProcessHeartbeat(3)

	td := app.GetTelemetry()
	if td.PacketsLost != 0 {
		t.Errorf("expected 0 lost after 1,2,3; got %d", td.PacketsLost)
	}
	if td.MsgCount != 3 {
		t.Errorf("expected msgCount 3, got %d", td.MsgCount)
	}

	// Gap: skip 4,5 → receive 6
	app.ProcessHeartbeat(6)

	td = app.GetTelemetry()
	if td.PacketsLost != 2 {
		t.Errorf("expected 2 lost after skip 4,5; got %d", td.PacketsLost)
	}

	// Another gap: skip 7,8,9 → receive 10
	app.ProcessHeartbeat(10)

	td = app.GetTelemetry()
	if td.PacketsLost != 5 {
		t.Errorf("expected 5 total lost; got %d", td.PacketsLost)
	}
	if td.MsgCount != 5 {
		t.Errorf("expected msgCount 5, got %d", td.MsgCount)
	}
}

// TestSensorDataOverwrite verifies that new sensor readings overwrite old ones.
func TestSensorDataOverwrite(t *testing.T) {
	app := backend.NewApp()

	// First reading
	app.ProcessScaledPressure(&common.MessageScaledPressure{
		PressAbs:    1013.25,
		Temperature: 2500,
	})

	td := app.GetTelemetry()
	if td.Pressure != 1013.25 {
		t.Errorf("first pressure: expected 1013.25, got %f", td.Pressure)
	}

	// Second reading overwrites
	app.ProcessScaledPressure(&common.MessageScaledPressure{
		PressAbs:    1015.00,
		Temperature: 2700,
	})

	td = app.GetTelemetry()
	if td.Pressure != 1015.00 {
		t.Errorf("updated pressure: expected 1015.00, got %f", td.Pressure)
	}
	if td.Temperature != 27.0 {
		t.Errorf("updated temperature: expected 27.0, got %f", td.Temperature)
	}
}

// TestIMUAndAttitudeCorrelation tests that IMU and Attitude data
// are stored independently without interference.
func TestIMUAndAttitudeCorrelation(t *testing.T) {
	app := backend.NewApp()

	app.ProcessScaledImu2(&common.MessageScaledImu2{
		Xacc: 100, Yacc: 200, Zacc: 300,
		Xgyro: 500, Ygyro: 600, Zgyro: 700,
	})

	app.ProcessAttitude(&common.MessageAttitude{
		Roll: 0.1, Pitch: 0.2, Yaw: 0.3,
	})

	td := app.GetTelemetry()

	// Verify IMU accel values are independent of attitude
	if td.AccelX != 1.0 {
		t.Errorf("accelX: expected 1.0, got %f", td.AccelX)
	}
	if td.AccelY != 2.0 {
		t.Errorf("accelY: expected 2.0, got %f", td.AccelY)
	}

	// Verify attitude is in degrees
	expectedRoll := float32(0.1) * 57.2958
	if td.Roll != expectedRoll {
		t.Errorf("roll: expected %f, got %f", expectedRoll, td.Roll)
	}
}
