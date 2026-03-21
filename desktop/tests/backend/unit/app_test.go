package unit

import (
	"testing"
	"time"

	"cansat-ground-station/backend"

	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
)

func TestNewApp(t *testing.T) {
	app := backend.NewApp()
	if app == nil {
		t.Fatal("NewApp returned nil")
	}
	td := app.GetTelemetry()
	if td.Mode != "Flight" {
		t.Errorf("expected mode 'Flight', got %q", td.Mode)
	}
	if td.State != "Launch Wait" {
		t.Errorf("expected state 'Launch Wait', got %q", td.State)
	}
	if app.IsConnected() {
		t.Error("new app should not be connected")
	}
	if td.MsgCount != 0 {
		t.Errorf("expected msgCount 0, got %d", td.MsgCount)
	}
}

func TestIsConnected(t *testing.T) {
	app := backend.NewApp()
	if app.IsConnected() {
		t.Error("new app should not be connected")
	}
	// Simulate connection state
	app.SetConnected(true)
	if !app.IsConnected() {
		t.Error("expected connected after setting flag")
	}
}

func TestGetMissionTime_Disconnected(t *testing.T) {
	app := backend.NewApp()
	result := app.GetMissionTime()
	if result != "00:00" {
		t.Errorf("expected '00:00' when disconnected, got %q", result)
	}
}

func TestGetMissionTime_Connected(t *testing.T) {
	app := backend.NewApp()
	app.SetConnected(true)
	app.SetMissionStart(time.Now().Add(-65 * time.Second)) // 1 min 5 sec ago

	result := app.GetMissionTime()
	if result != "01:05" {
		t.Errorf("expected '01:05', got %q", result)
	}
}

func TestDisconnect_WhenNotConnected(t *testing.T) {
	app := backend.NewApp()
	// Should not panic when disconnecting without connection
	app.Disconnect()
	if app.IsConnected() {
		t.Error("should remain disconnected")
	}
}

func TestDisconnect_WhenConnected(t *testing.T) {
	app := backend.NewApp()
	app.SetConnected(true)

	app.Disconnect()
	if app.IsConnected() {
		t.Error("should be disconnected after Disconnect()")
	}
}

func TestConnect_AlreadyConnected(t *testing.T) {
	app := backend.NewApp()
	app.SetConnected(true)

	err := app.Connect("/dev/null", 57600)
	if err == nil {
		t.Error("expected error when already connected")
	}
	if err.Error() != "already connected" {
		t.Errorf("expected 'already connected', got %q", err.Error())
	}
}

func TestConnect_InvalidPort(t *testing.T) {
	app := backend.NewApp()
	err := app.Connect("/dev/nonexistent_port_xyz", 57600)
	if err == nil {
		t.Error("expected error for invalid port")
	}
}

func TestSendCommand_NotConnected(t *testing.T) {
	app := backend.NewApp()
	err := app.SendCommand("CMD,1003,ON")
	if err == nil {
		t.Error("expected error when not connected")
	}
	if err.Error() != "not connected" {
		t.Errorf("expected 'not connected', got %q", err.Error())
	}
}

func TestListPorts(t *testing.T) {
	app := backend.NewApp()
	ports := app.ListPorts()
	// ListPorts should return a slice (possibly empty), never nil panic
	if ports == nil {
		t.Error("ListPorts returned nil, expected empty slice")
	}
}

func TestTelemetryData_Defaults(t *testing.T) {
	app := backend.NewApp()
	td := app.GetTelemetry()

	if td.Counter != 0 {
		t.Errorf("expected counter 0, got %d", td.Counter)
	}
	if td.Temperature != 0 {
		t.Errorf("expected temperature 0, got %f", td.Temperature)
	}
	if td.Pressure != 0 {
		t.Errorf("expected pressure 0, got %f", td.Pressure)
	}
	if td.Connected {
		t.Error("expected connected=false")
	}
}

// --- MAVLink message handling ---

func TestHandleMessage_ScaledPressure(t *testing.T) {
	app := backend.NewApp()
	app.ProcessScaledPressure(&common.MessageScaledPressure{
		PressAbs:    1013.25,
		Temperature: 2500, // 25.0°C
	})

	td := app.GetTelemetry()
	if td.Pressure != 1013.25 {
		t.Errorf("expected pressure 1013.25, got %f", td.Pressure)
	}
	if td.Temperature != 25.0 {
		t.Errorf("expected temperature 25.0, got %f", td.Temperature)
	}
}

func TestHandleMessage_VfrHud(t *testing.T) {
	app := backend.NewApp()
	app.ProcessVfrHud(&common.MessageVfrHud{
		Alt: 150.5,
	})

	td := app.GetTelemetry()
	if td.Altitude != 150.5 {
		t.Errorf("expected altitude 150.5, got %f", td.Altitude)
	}
}

func TestHandleMessage_SysStatus(t *testing.T) {
	app := backend.NewApp()
	app.ProcessSysStatus(&common.MessageSysStatus{
		VoltageBattery: 12500, // 12.5V
	})

	td := app.GetTelemetry()
	if td.Voltage != 12.5 {
		t.Errorf("expected voltage 12.5, got %f", td.Voltage)
	}
}

func TestHandleMessage_NamedValueFloat_Humidity(t *testing.T) {
	app := backend.NewApp()
	app.ProcessNamedValueFloat("HUMIDITY", 65.5)

	td := app.GetTelemetry()
	if td.Humidity != 65.5 {
		t.Errorf("expected humidity 65.5, got %f", td.Humidity)
	}
}

func TestHandleMessage_NamedValueFloat_Other(t *testing.T) {
	app := backend.NewApp()
	app.ProcessNamedValueFloat("OTHER", 42.0)

	td := app.GetTelemetry()
	if td.Humidity != 0 {
		t.Errorf("expected humidity unchanged (0), got %f", td.Humidity)
	}
}

func TestHandleMessage_ScaledImu2(t *testing.T) {
	app := backend.NewApp()
	app.ProcessScaledImu2(&common.MessageScaledImu2{
		Xacc:  980,  // 9.8 m/s²
		Yacc:  0,
		Zacc:  -980,
		Xgyro: 1745, // ~100 deg/s
		Ygyro: 0,
		Zgyro: 0,
	})

	td := app.GetTelemetry()

	expectedAccelX := float32(980) / 100.0
	if td.AccelX != expectedAccelX {
		t.Errorf("expected accelX %f, got %f", expectedAccelX, td.AccelX)
	}

	expectedAccelZ := float32(-980) / 100.0
	if td.AccelZ != expectedAccelZ {
		t.Errorf("expected accelZ %f, got %f", expectedAccelZ, td.AccelZ)
	}

	expectedGyroX := float32(1745) / 17.4533
	if td.GyroX != expectedGyroX {
		t.Errorf("expected gyroX %f, got %f", expectedGyroX, td.GyroX)
	}
}

func TestHandleMessage_Attitude(t *testing.T) {
	app := backend.NewApp()
	app.ProcessAttitude(&common.MessageAttitude{
		Roll:  0.5236,  // ~30 degrees
		Pitch: -0.2618, // ~-15 degrees
		Yaw:   1.5708,  // ~90 degrees
	})

	td := app.GetTelemetry()

	expectedRoll := float32(0.5236) * 57.2958
	if td.Roll != expectedRoll {
		t.Errorf("expected roll %f, got %f", expectedRoll, td.Roll)
	}

	expectedPitch := float32(-0.2618) * 57.2958
	if td.Pitch != expectedPitch {
		t.Errorf("expected pitch %f, got %f", expectedPitch, td.Pitch)
	}

	expectedYaw := float32(1.5708) * 57.2958
	if td.Yaw != expectedYaw {
		t.Errorf("expected yaw %f, got %f", expectedYaw, td.Yaw)
	}
}

func TestHandleMessage_GpsRawInt(t *testing.T) {
	app := backend.NewApp()
	app.ProcessGpsRawInt(&common.MessageGpsRawInt{
		Lat:               413111000, // 41.3111
		Lon:               692797000, // 69.2797
		Alt:               450000,    // 450m
		SatellitesVisible: 12,
		FixType:           3, // 3D fix
	})

	td := app.GetTelemetry()

	if td.GpsLat != 41.3111 {
		t.Errorf("expected lat 41.3111, got %f", td.GpsLat)
	}
	if td.GpsLon != 69.2797 {
		t.Errorf("expected lon 69.2797, got %f", td.GpsLon)
	}
	if td.GpsAlt != 450.0 {
		t.Errorf("expected alt 450.0, got %f", td.GpsAlt)
	}
	if td.GpsSats != 12 {
		t.Errorf("expected sats 12, got %d", td.GpsSats)
	}
	if td.GpsFix != 3 {
		t.Errorf("expected fix 3, got %d", td.GpsFix)
	}
}

func TestHandleMessage_Heartbeat(t *testing.T) {
	app := backend.NewApp()
	app.ProcessHeartbeat(5)

	td := app.GetTelemetry()

	if td.Counter != 5 {
		t.Errorf("expected counter 5, got %d", td.Counter)
	}
	if td.MsgCount != 1 {
		t.Errorf("expected msgCount 1, got %d", td.MsgCount)
	}
	if td.Timestamp == "" {
		t.Error("expected timestamp to be set")
	}
}

func TestPacketLossTracking(t *testing.T) {
	app := backend.NewApp()

	// First heartbeat: counter=1
	app.ProcessHeartbeat(1)
	// Second heartbeat: counter=5 (skipped 2,3,4 = 3 lost)
	app.ProcessHeartbeat(5)

	td := app.GetTelemetry()

	if td.PacketsLost != 3 {
		t.Errorf("expected 3 packets lost, got %d", td.PacketsLost)
	}
}

func TestPacketLossTracking_NoLoss(t *testing.T) {
	app := backend.NewApp()

	app.ProcessHeartbeat(1)
	app.ProcessHeartbeat(2)
	app.ProcessHeartbeat(3)

	td := app.GetTelemetry()

	if td.PacketsLost != 0 {
		t.Errorf("expected 0 packets lost, got %d", td.PacketsLost)
	}
	if td.MsgCount != 3 {
		t.Errorf("expected msgCount 3, got %d", td.MsgCount)
	}
}

func TestConcurrentAccess(t *testing.T) {
	app := backend.NewApp()
	app.SetConnected(true)
	app.SetMissionStart(time.Now())

	done := make(chan struct{})

	// Concurrent reads
	go func() {
		for i := 0; i < 100; i++ {
			_ = app.IsConnected()
			_ = app.GetMissionTime()
		}
		done <- struct{}{}
	}()

	// Concurrent writes
	go func() {
		for i := 0; i < 100; i++ {
			app.ProcessHeartbeat(uint32(i))
		}
		done <- struct{}{}
	}()

	<-done
	<-done
}
