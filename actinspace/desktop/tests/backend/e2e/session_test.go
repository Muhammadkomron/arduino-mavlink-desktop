package e2e

import (
	"sync"
	"testing"
	"time"

	"cansat-ground-station/backend"

	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
)

// TestFullSessionLifecycle simulates a complete ground station session:
// init → verify defaults → process messages → verify state → disconnect → verify reset
func TestFullSessionLifecycle(t *testing.T) {
	// Phase 1: Initialize
	app := backend.NewApp()

	td := app.GetTelemetry()
	if td.Mode != "Flight" {
		t.Fatalf("expected initial mode 'Flight', got %q", td.Mode)
	}
	if td.State != "Launch Wait" {
		t.Fatalf("expected initial state 'Launch Wait', got %q", td.State)
	}
	if app.IsConnected() {
		t.Fatal("should not be connected initially")
	}

	// Phase 2: Simulate connection (without real serial port)
	// We can't call Connect() without a real port, but we can test
	// the disconnect path and verify state management

	// Phase 3: Process a full telemetry cycle
	app.ProcessScaledPressure(&common.MessageScaledPressure{
		PressAbs:    1013.25,
		Temperature: 2200,
	})
	app.ProcessVfrHud(&common.MessageVfrHud{Alt: 100.0})
	app.ProcessSysStatus(&common.MessageSysStatus{VoltageBattery: 11800})
	app.ProcessGpsRawInt(&common.MessageGpsRawInt{
		Lat: 413111000, Lon: 692797000, Alt: 100000,
		SatellitesVisible: 8, FixType: 3,
	})
	app.ProcessScaledImu2(&common.MessageScaledImu2{
		Xacc: 50, Yacc: -30, Zacc: 981,
		Xgyro: 100, Ygyro: -50, Zgyro: 25,
	})
	app.ProcessAttitude(&common.MessageAttitude{
		Roll: 0.05, Pitch: -0.02, Yaw: 3.14,
	})
	app.ProcessNamedValueFloat("HUMIDITY", 45.0)
	app.ProcessHeartbeat(1)

	// Phase 4: Verify full telemetry state
	td = app.GetTelemetry()

	if td.Pressure != 1013.25 {
		t.Errorf("pressure: got %f, want 1013.25", td.Pressure)
	}
	if td.Temperature != 22.0 {
		t.Errorf("temperature: got %f, want 22.0", td.Temperature)
	}
	if td.Altitude != 100.0 {
		t.Errorf("altitude: got %f, want 100.0", td.Altitude)
	}
	if td.Voltage != 11.8 {
		t.Errorf("voltage: got %f, want 11.8", td.Voltage)
	}
	if td.GpsLat != 41.3111 {
		t.Errorf("gpsLat: got %f, want 41.3111", td.GpsLat)
	}
	if td.GpsSats != 8 {
		t.Errorf("gpsSats: got %d, want 8", td.GpsSats)
	}
	if td.Humidity != 45.0 {
		t.Errorf("humidity: got %f, want 45.0", td.Humidity)
	}
	if td.MsgCount != 1 {
		t.Errorf("msgCount: got %d, want 1", td.MsgCount)
	}
	if td.Counter != 1 {
		t.Errorf("counter: got %d, want 1", td.Counter)
	}
	if td.Timestamp == "" {
		t.Error("timestamp should be set after heartbeat")
	}

	// Phase 5: Disconnect
	app.Disconnect()
	if app.IsConnected() {
		t.Error("should be disconnected after Disconnect()")
	}
}

// TestConcurrentTelemetryProcessing simulates multiple goroutines sending
// different message types concurrently — verifies no data races.
func TestConcurrentTelemetryProcessing(t *testing.T) {
	app := backend.NewApp()

	var wg sync.WaitGroup
	iterations := 200

	// Goroutine 1: pressure + temperature
	wg.Add(1)
	go func() {
		defer wg.Done()
		for i := 0; i < iterations; i++ {
			app.ProcessScaledPressure(&common.MessageScaledPressure{
				PressAbs:    float32(1000 + i),
				Temperature: int16(2000 + i),
			})
		}
	}()

	// Goroutine 2: altitude
	wg.Add(1)
	go func() {
		defer wg.Done()
		for i := 0; i < iterations; i++ {
			app.ProcessVfrHud(&common.MessageVfrHud{
				Alt: float32(i),
			})
		}
	}()

	// Goroutine 3: heartbeats
	wg.Add(1)
	go func() {
		defer wg.Done()
		for i := 0; i < iterations; i++ {
			app.ProcessHeartbeat(uint32(i + 1))
		}
	}()

	// Goroutine 4: GPS
	wg.Add(1)
	go func() {
		defer wg.Done()
		for i := 0; i < iterations; i++ {
			app.ProcessGpsRawInt(&common.MessageGpsRawInt{
				Lat: int32(410000000 + i*1000),
				Lon: int32(690000000 + i*1000),
				Alt: int32(i * 1000),
			})
		}
	}()

	// Goroutine 5: IMU + Attitude
	wg.Add(1)
	go func() {
		defer wg.Done()
		for i := 0; i < iterations; i++ {
			app.ProcessScaledImu2(&common.MessageScaledImu2{
				Xacc: int16(i), Yacc: int16(i), Zacc: int16(i),
			})
			app.ProcessAttitude(&common.MessageAttitude{
				Roll: float32(i) * 0.01,
			})
		}
	}()

	// Goroutine 6: concurrent reads
	wg.Add(1)
	go func() {
		defer wg.Done()
		for i := 0; i < iterations; i++ {
			_ = app.IsConnected()
			_ = app.GetMissionTime()
			_ = app.GetTelemetry()
		}
	}()

	wg.Wait()

	// If we get here without a race condition panic, the test passes
	td := app.GetTelemetry()
	if td.MsgCount != iterations {
		t.Errorf("expected %d heartbeats processed, got %d", iterations, td.MsgCount)
	}
}

// TestConnectDisconnectCycle tests rapid connect/disconnect doesn't corrupt state.
func TestConnectDisconnectCycle(t *testing.T) {
	app := backend.NewApp()

	for i := 0; i < 10; i++ {
		// Try connecting to invalid port (should error)
		err := app.Connect("/dev/nonexistent_test_port", 57600)
		if err == nil {
			t.Fatal("expected error connecting to invalid port")
		}

		// Should still be disconnected
		if app.IsConnected() {
			t.Fatalf("iteration %d: should not be connected after failed connect", i)
		}

		// Disconnect should be safe even when not connected
		app.Disconnect()

		if app.IsConnected() {
			t.Fatalf("iteration %d: should not be connected after disconnect", i)
		}
	}
}

// TestMissionTimerAccuracy tests the mission timer produces correct format.
func TestMissionTimerAccuracy(t *testing.T) {
	app := backend.NewApp()

	// When disconnected, mission time should be 00:00
	if mt := app.GetMissionTime(); mt != "00:00" {
		t.Errorf("expected '00:00' when disconnected, got %q", mt)
	}

	// Simulate connected state with a known start time
	app.SetConnected(true)
	app.SetMissionStart(time.Now().Add(-125 * time.Second)) // 2 min 5 sec ago

	mt := app.GetMissionTime()
	if mt != "02:05" {
		t.Errorf("expected '02:05', got %q", mt)
	}
}

// TestSendCommandRequiresConnection verifies commands can't be sent when disconnected.
func TestSendCommandRequiresConnection(t *testing.T) {
	app := backend.NewApp()

	commands := []string{
		"CMD,1003,ON",
		"CMD,1003,OFF",
		"CMD,1003,SIM,1",
		"CMD,1003,SIM,0",
		"CMD,1003,SIM,2",
		"CMD,1003,CAL",
		"CMD,1003,ST,12:00:00",
		"CMD,1003,SD,21.03.2026",
	}

	for _, cmd := range commands {
		err := app.SendCommand(cmd)
		if err == nil {
			t.Errorf("expected error sending %q when disconnected", cmd)
		}
		if err.Error() != "not connected" {
			t.Errorf("expected 'not connected' error for %q, got %q", cmd, err.Error())
		}
	}
}
