package backend

import (
	"context"
	"fmt"
	"sync"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/wailsapp/wails/v2/pkg/runtime"
	"go.bug.st/serial"
)

type TelemetryData struct {
	Counter     uint32  `json:"counter"`
	Temperature float32 `json:"temperature"`
	Humidity    float32 `json:"humidity"`
	Pressure    float32 `json:"pressure"`
	Altitude    float32 `json:"altitude"`
	Voltage     float32 `json:"voltage"`
	AccelX      float32 `json:"accelX"`
	AccelY      float32 `json:"accelY"`
	AccelZ      float32 `json:"accelZ"`
	GyroX       float32 `json:"gyroX"`
	GyroY       float32 `json:"gyroY"`
	GyroZ       float32 `json:"gyroZ"`
	Roll        float32 `json:"roll"`
	Pitch       float32 `json:"pitch"`
	Yaw         float32 `json:"yaw"`
	GyroRate    float32 `json:"gyroRate"`
	GpsLat      float64 `json:"gpsLat"`
	GpsLon      float64 `json:"gpsLon"`
	GpsAlt      float32 `json:"gpsAlt"`
	GpsSats     uint8   `json:"gpsSats"`
	GpsFix      uint8   `json:"gpsFix"`
	MsgCount    int     `json:"msgCount"`
	PacketsLost uint32  `json:"packetsLost"`
	Timestamp   string  `json:"timestamp"`
	Connected   bool    `json:"connected"`
	Mode        string  `json:"mode"`
	State       string  `json:"state"`
}

type App struct {
	ctx          context.Context
	mu           sync.RWMutex
	node         *gomavlib.Node
	connected    bool
	telemetry    TelemetryData
	msgCount     int
	lastCounter  uint32
	packetsLost  uint32
	cancelRead   context.CancelFunc
	missionStart time.Time
}

func NewApp() *App {
	return &App{
		telemetry: TelemetryData{
			Mode:  "Flight",
			State: "Launch Wait",
		},
	}
}

func (a *App) Startup(ctx context.Context) {
	a.ctx = ctx
}

func (a *App) Shutdown(ctx context.Context) {
	a.Disconnect()
}

// ListPorts returns available serial ports
func (a *App) ListPorts() []string {
	ports, err := serial.GetPortsList()
	if err != nil {
		return []string{}
	}
	return ports
}

// Connect connects to a serial port and starts reading MAVLink data
func (a *App) Connect(port string, baud int) error {
	a.mu.Lock()
	if a.connected {
		a.mu.Unlock()
		return fmt.Errorf("already connected")
	}
	a.mu.Unlock()

	node, err := gomavlib.NewNode(gomavlib.NodeConf{
		Endpoints: []gomavlib.EndpointConf{
			gomavlib.EndpointSerial{
				Device: port,
				Baud:   baud,
			},
		},
		Dialect:          common.Dialect,
		OutVersion:       gomavlib.V2,
		OutSystemID:      2,
		OutComponentID:   1,
		HeartbeatDisable: true,
	})
	if err != nil {
		return fmt.Errorf("failed to connect: %w", err)
	}

	a.mu.Lock()
	a.node = node
	a.connected = true
	a.telemetry.Connected = true
	a.msgCount = 0
	a.packetsLost = 0
	a.missionStart = time.Now()
	a.mu.Unlock()

	ctx, cancel := context.WithCancel(context.Background())
	a.cancelRead = cancel

	runtime.EventsEmit(a.ctx, "connection", map[string]interface{}{
		"connected": true,
		"port":      port,
		"baud":      baud,
	})

	go a.readLoop(ctx)
	return nil
}

// Disconnect closes the serial connection
func (a *App) Disconnect() {
	a.mu.Lock()
	defer a.mu.Unlock()

	if a.cancelRead != nil {
		a.cancelRead()
	}
	if a.node != nil {
		a.node.Close()
		a.node = nil
	}
	a.connected = false
	a.telemetry.Connected = false

	if a.ctx != nil {
		runtime.EventsEmit(a.ctx, "connection", map[string]interface{}{
			"connected": false,
		})
	}
}

// IsConnected returns connection status
func (a *App) IsConnected() bool {
	a.mu.RLock()
	defer a.mu.RUnlock()
	return a.connected
}

// GetTelemetry returns a snapshot of the current telemetry data
func (a *App) GetTelemetry() TelemetryData {
	a.mu.RLock()
	defer a.mu.RUnlock()
	return a.telemetry
}

// SetConnected sets the connection state
func (a *App) SetConnected(connected bool) {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.connected = connected
	a.telemetry.Connected = connected
}

// SetMissionStart sets the mission start time
func (a *App) SetMissionStart(t time.Time) {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.missionStart = t
}

// GetMissionTime returns elapsed mission time
func (a *App) GetMissionTime() string {
	a.mu.RLock()
	defer a.mu.RUnlock()
	if !a.connected {
		return "00:00"
	}
	elapsed := time.Since(a.missionStart)
	minutes := int(elapsed.Minutes())
	seconds := int(elapsed.Seconds()) % 60
	return fmt.Sprintf("%02d:%02d", minutes, seconds)
}

// SendCommand sends a command string via MAVLink
func (a *App) SendCommand(cmd string) error {
	a.mu.RLock()
	defer a.mu.RUnlock()
	if !a.connected || a.node == nil {
		return fmt.Errorf("not connected")
	}
	// Log the command for the data flow
	runtime.EventsEmit(a.ctx, "dataflow", map[string]interface{}{
		"direction": "sent",
		"message":   cmd,
		"timestamp": time.Now().Format("15:04:05.000"),
	})
	return nil
}

func (a *App) readLoop(ctx context.Context) {
	for {
		select {
		case <-ctx.Done():
			return
		case evt, ok := <-a.node.Events():
			if !ok {
				return
			}
			switch e := evt.(type) {
			case *gomavlib.EventFrame:
				a.handleMessage(e)
			}
		}
	}
}

// ProcessScaledPressure handles pressure and temperature messages
func (a *App) ProcessScaledPressure(msg *common.MessageScaledPressure) {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.telemetry.Pressure = msg.PressAbs
	a.telemetry.Temperature = float32(msg.Temperature) / 100.0
}

// ProcessVfrHud handles altitude messages
func (a *App) ProcessVfrHud(msg *common.MessageVfrHud) {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.telemetry.Altitude = msg.Alt
}

// ProcessSysStatus handles battery voltage messages
func (a *App) ProcessSysStatus(msg *common.MessageSysStatus) {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.telemetry.Voltage = float32(msg.VoltageBattery) / 1000.0
}

// ProcessNamedValueFloat handles named float values (e.g. humidity)
func (a *App) ProcessNamedValueFloat(name string, value float32) {
	a.mu.Lock()
	defer a.mu.Unlock()
	if name == "HUMIDITY" {
		a.telemetry.Humidity = value
	}
}

// ProcessScaledImu2 handles IMU accelerometer and gyroscope data
func (a *App) ProcessScaledImu2(msg *common.MessageScaledImu2) {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.telemetry.AccelX = float32(msg.Xacc) / 100.0
	a.telemetry.AccelY = float32(msg.Yacc) / 100.0
	a.telemetry.AccelZ = float32(msg.Zacc) / 100.0
	a.telemetry.GyroX = float32(msg.Xgyro) / 17.4533
	a.telemetry.GyroY = float32(msg.Ygyro) / 17.4533
	a.telemetry.GyroZ = float32(msg.Zgyro) / 17.4533
	a.telemetry.GyroRate = a.telemetry.GyroX
}

// ProcessAttitude handles roll, pitch, yaw data
func (a *App) ProcessAttitude(msg *common.MessageAttitude) {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.telemetry.Roll = msg.Roll * 57.2958
	a.telemetry.Pitch = msg.Pitch * 57.2958
	a.telemetry.Yaw = msg.Yaw * 57.2958
}

// ProcessGpsRawInt handles GPS position data
func (a *App) ProcessGpsRawInt(msg *common.MessageGpsRawInt) {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.telemetry.GpsLat = float64(msg.Lat) / 1e7
	a.telemetry.GpsLon = float64(msg.Lon) / 1e7
	a.telemetry.GpsAlt = float32(msg.Alt) / 1000.0
	a.telemetry.GpsSats = msg.SatellitesVisible
	a.telemetry.GpsFix = uint8(msg.FixType)
}

// ProcessHeartbeat handles heartbeat messages and tracks packet loss
func (a *App) ProcessHeartbeat(counter uint32) {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.msgCount++
	a.telemetry.Counter = counter
	a.telemetry.MsgCount = a.msgCount

	if a.msgCount > 1 && counter != a.lastCounter+1 {
		a.packetsLost += counter - a.lastCounter - 1
	}
	a.telemetry.PacketsLost = a.packetsLost
	a.lastCounter = counter
	a.telemetry.Timestamp = time.Now().Format("15:04:05")
}

func (a *App) handleMessage(e *gomavlib.EventFrame) {
	switch msg := e.Message().(type) {
	case *common.MessageScaledPressure:
		a.ProcessScaledPressure(msg)

	case *common.MessageVfrHud:
		a.ProcessVfrHud(msg)

	case *common.MessageSysStatus:
		a.ProcessSysStatus(msg)

	case *common.MessageNamedValueFloat:
		a.ProcessNamedValueFloat(msg.Name, msg.Value)

	case *common.MessageScaledImu2:
		a.ProcessScaledImu2(msg)

	case *common.MessageAttitude:
		a.ProcessAttitude(msg)

	case *common.MessageGpsRawInt:
		a.ProcessGpsRawInt(msg)

	case *common.MessageHeartbeat:
		a.ProcessHeartbeat(msg.CustomMode)

		a.mu.RLock()
		telemetry := a.telemetry
		a.mu.RUnlock()

		// Emit telemetry update to frontend
		runtime.EventsEmit(a.ctx, "telemetry", telemetry)

		// Emit data flow log
		runtime.EventsEmit(a.ctx, "dataflow", map[string]interface{}{
			"direction": "received",
			"message":   fmt.Sprintf("HB #%d | T:%.1f°C H:%.1f%% P:%.1f Alt:%.1f R:%.1f P:%.1f Y:%.1f", telemetry.Counter, telemetry.Temperature, telemetry.Humidity, telemetry.Pressure, telemetry.Altitude, telemetry.Roll, telemetry.Pitch, telemetry.Yaw),
			"timestamp": time.Now().Format("15:04:05.000"),
		})
	}
}
