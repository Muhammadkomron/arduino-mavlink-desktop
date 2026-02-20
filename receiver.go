package main

import (
	"fmt"
	"log"
	"strings"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
)

// ANSI escape codes
const (
	clearScreen = "\033[2J"
	homePos     = "\033[H"
	hideCursor  = "\033[?25l"
	showCursor  = "\033[?25h"
)

func makeLine(content string, width int) string {
	// Pad or truncate content to exactly width characters
	// Handle string length properly (some chars like ° might be multi-byte)
	runes := []rune(content)
	if len(runes) > width {
		content = string(runes[:width])
	} else if len(runes) < width {
		content = content + strings.Repeat(" ", width-len(runes))
	}

	return "║" + content + "║"
}

func drawTable(counter uint32, temperature, humidity, pressure, altitude, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, roll, pitch, yaw float32, messageCount int, lastUpdate time.Time, packetsLost uint32) {
	// Move cursor to home and clear
	fmt.Print(homePos)

	// Box drawing - wider for horizontal layout (80 characters)
	const width = 78

	border := "╔" + strings.Repeat("═", width) + "╗"
	fmt.Println(border)

	// Title
	content := " MAVLink LoRa Receiver - Live Telemetry               /dev/cu.usbserial-0001"
	fmt.Println(makeLine(content, width))

	fmt.Println("╠" + strings.Repeat("═", width) + "╣")

	// Status line with counter, messages, packets lost, time
	updateStr := lastUpdate.Format("15:04:05")
	lostStr := "None"
	if packetsLost > 0 {
		lostStr = fmt.Sprintf("%d", packetsLost)
	}
	content = fmt.Sprintf(" Count: %-6d  Msgs: %-6d  Lost: %-6s  Time: %s", counter, messageCount, lostStr, updateStr)
	fmt.Println(makeLine(content, width))

	fmt.Println("╠" + strings.Repeat("═", width) + "╣")

	// Environmental Sensors - Horizontal
	content = " ENVIRONMENT │ Temp    Humidity  Pressure   Altitude"
	fmt.Println(makeLine(content, width))

	content = fmt.Sprintf(" BMP+AHT     │ %.1f°C  %.1f%%    %.1fhPa  %.1fm MSL",
		temperature, humidity, pressure, altitude)
	fmt.Println(makeLine(content, width))

	fmt.Println("╠" + strings.Repeat("═", width) + "╣")

	// IMU - Accelerometer - Horizontal
	content = " ACCEL (m/s²)│   X        Y        Z"
	fmt.Println(makeLine(content, width))

	content = fmt.Sprintf(" MPU6050     │ %6.2f   %6.2f   %6.2f", accelX, accelY, accelZ)
	fmt.Println(makeLine(content, width))

	fmt.Println("╟" + strings.Repeat("─", width) + "╢")

	// IMU - Gyroscope - Horizontal
	content = " GYRO (°/s)  │   X        Y        Z"
	fmt.Println(makeLine(content, width))

	content = fmt.Sprintf(" MPU6050     │ %6.2f   %6.2f   %6.2f", gyroX, gyroY, gyroZ)
	fmt.Println(makeLine(content, width))

	fmt.Println("╠" + strings.Repeat("═", width) + "╣")

	// Orientation - Horizontal
	content = " ORIENTATION │  Roll     Pitch     Yaw"
	fmt.Println(makeLine(content, width))

	content = fmt.Sprintf(" RPY (deg)   │ %6.1f°  %6.1f°  %6.1f°", roll, pitch, yaw)
	fmt.Println(makeLine(content, width))

	fmt.Println("╚" + strings.Repeat("═", width) + "╝")
	fmt.Println("\nPress Ctrl+C to exit")
}

func main() {
	// Clear screen and hide cursor
	fmt.Print(clearScreen)
	fmt.Print(hideCursor)
	defer fmt.Print(showCursor) // Show cursor on exit

	// Create MAVLink node
	node, err := gomavlib.NewNode(gomavlib.NodeConf{
		Endpoints: []gomavlib.EndpointConf{
			gomavlib.EndpointSerial{
				Device: "/dev/cu.usbserial-0001", // Receiver LoRa USB
				Baud:   57600,
			},
		},
		Dialect:          common.Dialect,
		OutVersion:       gomavlib.V2,
		OutSystemID:      2,
		OutComponentID:   1,
		HeartbeatDisable: true,
	})
	if err != nil {
		log.Fatal("Failed to create MAVLink node:", err)
	}
	defer node.Close()

	messageCount := 0
	var lastCounter uint32
	var counter uint32
	var temperature float32
	var pressure float32
	var altitude float32
	var humidity float32
	var accelX float32
	var accelY float32
	var accelZ float32
	var gyroX float32
	var gyroY float32
	var gyroZ float32
	var roll float32
	var pitch float32
	var yaw float32
	var lastUpdate time.Time
	var packetsLost uint32

	// Draw initial table
	drawTable(counter, temperature, humidity, pressure, altitude, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, roll, pitch, yaw, messageCount, time.Now(), packetsLost)

	// Read messages
	for evt := range node.Events() {
		switch e := evt.(type) {
		case *gomavlib.EventFrame:
			// Handle different message types
			switch msg := e.Message().(type) {

			case *common.MessageScaledPressure:
				// ArduPilot pressure data
				pressure = msg.PressAbs
				temperature = float32(msg.Temperature) / 100.0 // Convert from centi-degrees
				lastUpdate = time.Now()

			case *common.MessageVfrHud:
				// ArduPilot VFR HUD with altitude
				altitude = msg.Alt
				lastUpdate = time.Now()

			case *common.MessageNamedValueFloat:
				// Custom telemetry values
				if msg.Name == "HUMIDITY" {
					humidity = msg.Value
					lastUpdate = time.Now()
				}

			case *common.MessageScaledImu2:
				// IMU data from MPU6050 (accelerometer + gyroscope)
				accelX = float32(msg.Xacc) / 100.0 // Convert from mG to m/s²
				accelY = float32(msg.Yacc) / 100.0
				accelZ = float32(msg.Zacc) / 100.0
				gyroX = float32(msg.Xgyro) / 17.4533  // Convert from millirad/s to deg/s
				gyroY = float32(msg.Ygyro) / 17.4533
				gyroZ = float32(msg.Zgyro) / 17.4533
				lastUpdate = time.Now()

			case *common.MessageAttitude:
				// Attitude data (roll, pitch, yaw)
				roll = msg.Roll * 57.2958   // Convert radians to degrees
				pitch = msg.Pitch * 57.2958 // Convert radians to degrees
				yaw = msg.Yaw * 57.2958     // Convert radians to degrees
				lastUpdate = time.Now()

			case *common.MessageHeartbeat:
				// Heartbeat with counter
				messageCount++
				counter = msg.CustomMode
				lastUpdate = time.Now()

				// Check for missed packets
				if messageCount > 1 && counter != lastCounter+1 {
					packetsLost += counter - lastCounter - 1
				}

				lastCounter = counter

				// Redraw table on heartbeat (10Hz)
				drawTable(counter, temperature, humidity, pressure, altitude, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, roll, pitch, yaw, messageCount, lastUpdate, packetsLost)
			}
		}
	}
}