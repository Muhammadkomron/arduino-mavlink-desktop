package backend

import (
	"context"
	"fmt"
	"os"
	"os/exec"
	"strings"
	"sync"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/wailsapp/wails/v2/pkg/runtime"
	"go.bug.st/serial"
)

const defaultTeamID = "1003"

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
	MagX        float32 `json:"magX"`
	MagY        float32 `json:"magY"`
	MagZ        float32 `json:"magZ"`
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
	teamID       string

	// Child window process tracking (one per view)
	childMu   sync.Mutex
	children  map[string]*exec.Cmd
}

func NewApp() *App {
	teamID := os.Getenv("TEAM_ID")
	if teamID == "" {
		teamID = defaultTeamID
	}
	return &App{
		teamID: teamID,
		children: make(map[string]*exec.Cmd),
		telemetry: TelemetryData{
			Mode:  "Flight",
			State: "Launch Wait",
		},
	}
}

// GetTeamID returns the configured team ID
func (a *App) GetTeamID() string {
	return a.teamID
}

func (a *App) Startup(ctx context.Context) {
	a.ctx = ctx
	StartSSEServer()
}

// StartupChild is for standalone child windows — no SSE server, no serial.
func (a *App) StartupChild(ctx context.Context) {
	a.ctx = ctx
}

// GetViewMode returns the CANSAT_VIEW env var so the frontend knows which
// standalone view to render (empty string = main dashboard).
func (a *App) GetViewMode() string {
	return os.Getenv("CANSAT_VIEW")
}

// ToggleWindow opens the view if not open, or closes it if already open.
// Returns true if the window is now open, false if it was closed.
func (a *App) ToggleWindow(view string) bool {
	a.childMu.Lock()
	defer a.childMu.Unlock()

	// If already open and still running, kill it
	if cmd, ok := a.children[view]; ok {
		if cmd.Process != nil {
			cmd.Process.Kill()
			cmd.Wait()
		}
		delete(a.children, view)
		return false
	}

	// Spawn a new child window
	exe, err := os.Executable()
	if err != nil {
		fmt.Println("ToggleWindow: failed to get executable path:", err)
		return false
	}

	cmd := exec.Command(exe)
	// Build a clean env: carry over parent env but strip Wails dev-mode
	// vars/flags that cause the child to try binding port 34115.
	cleanEnv := []string{"CANSAT_VIEW=" + view}
	for _, e := range os.Environ() {
		// Skip Wails dev-mode vars that cause port conflicts
		if len(e) > 0 &&
			!strings.HasPrefix(e, "WAILS_") &&
			!strings.HasPrefix(e, "devserver=") &&
			!strings.HasPrefix(e, "frontenddevserverurl=") &&
			!strings.HasPrefix(e, "assetdir=") &&
			!strings.HasPrefix(e, "loglevel=") {
			cleanEnv = append(cleanEnv, e)
		}
	}
	cmd.Env = cleanEnv
	// Don't pipe child output to parent — suppresses Wails dev server warnings
	if err := cmd.Start(); err != nil {
		fmt.Println("ToggleWindow: failed to start child:", err)
		return false
	}

	a.children[view] = cmd

	// Monitor in background: auto-cleanup when window is closed by user
	go func() {
		cmd.Wait()
		a.childMu.Lock()
		// Only delete if it's still the same cmd (not replaced by a new one)
		if curr, ok := a.children[view]; ok && curr == cmd {
			delete(a.children, view)
		}
		a.childMu.Unlock()
	}()

	return true
}

// IsWindowOpen checks if a child window for the given view is currently running.
func (a *App) IsWindowOpen(view string) bool {
	a.childMu.Lock()
	defer a.childMu.Unlock()

	cmd, ok := a.children[view]
	if !ok {
		return false
	}
	// Check if process is still alive
	if cmd.ProcessState != nil && cmd.ProcessState.Exited() {
		delete(a.children, view)
		return false
	}
	return true
}

// OpenWindow is kept for backward compatibility — calls ToggleWindow.
func (a *App) OpenWindow(view string) {
	a.ToggleWindow(view)
}

func (a *App) Shutdown(ctx context.Context) {
	a.Disconnect()
	a.CloseAllChildren()
}

// SetTheme broadcasts a theme change to all child windows via SSE
func (a *App) SetTheme(theme string) {
	BroadcastTheme(theme)
}

// CloseAllChildren kills all spawned child window processes
func (a *App) CloseAllChildren() {
	a.childMu.Lock()
	defer a.childMu.Unlock()
	for view, cmd := range a.children {
		if cmd.Process != nil {
			cmd.Process.Kill()
			cmd.Wait()
		}
		delete(a.children, view)
	}
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

// ResetMPU sends a CAL command to the transmitter and resets local orientation to zero.
// This provides immediate visual feedback on the ground station while the transmitter recalibrates.
func (a *App) ResetMPU() error {
	cmd := fmt.Sprintf("CMD,%s,CAL", a.teamID)
	err := a.SendCommand(cmd)
	if err != nil {
		return err
	}
	// Also reset local telemetry orientation so the 3D view snaps to zero immediately
	a.mu.Lock()
	a.telemetry.Roll = 0
	a.telemetry.Pitch = 0
	a.telemetry.Yaw = 0
	a.mu.Unlock()
	return nil
}

// SendCommand sends a command string via MAVLink STATUSTEXT message.
// Works universally for any command: CAL, CX ON/OFF, SIM, ST, SD, etc.
// The transmitter parses the text to determine the action.
func (a *App) SendCommand(cmd string) error {
	a.mu.RLock()
	defer a.mu.RUnlock()
	if !a.connected || a.node == nil {
		return fmt.Errorf("not connected")
	}

	// Send command as MAVLink STATUSTEXT (max 50 chars — all our commands fit)
	err := a.node.WriteMessageAll(&common.MessageStatustext{
		Severity: 6, // MAV_SEVERITY_INFO
		Text:     cmd,
	})
	if err != nil {
		return fmt.Errorf("failed to send command: %w", err)
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
	a.telemetry.MagX = float32(msg.Xmag) / 10.0
	a.telemetry.MagY = float32(msg.Ymag) / 10.0
	a.telemetry.MagZ = float32(msg.Zmag) / 10.0
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

		// Emit telemetry update to frontend and SSE clients
		runtime.EventsEmit(a.ctx, "telemetry", telemetry)
		BroadcastTelemetry(telemetry)

		// Emit data flow log
		runtime.EventsEmit(a.ctx, "dataflow", map[string]interface{}{
			"direction": "received",
			"message":   fmt.Sprintf("HB #%d | T:%.1f°C H:%.1f%% P:%.1f Alt:%.1f R:%.1f P:%.1f Y:%.1f", telemetry.Counter, telemetry.Temperature, telemetry.Humidity, telemetry.Pressure, telemetry.Altitude, telemetry.Roll, telemetry.Pitch, telemetry.Yaw),
			"timestamp": time.Now().Format("15:04:05.000"),
		})
	}
}
