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

func makeLine(content string) string {
	// Each line must be exactly: ║ + 64 chars + ║ = 66 total
	// Pad or truncate content to exactly 64 characters
	const width = 64

	// Handle string length properly (some chars like ° might be multi-byte)
	runes := []rune(content)
	if len(runes) > width {
		content = string(runes[:width])
	} else if len(runes) < width {
		content = content + strings.Repeat(" ", width-len(runes))
	}

	return "║" + content + "║"
}

func drawTable(counter uint32, temperature, humidity, pressure, altitude float32, messageCount int, lastUpdate time.Time, packetsLost uint32) {
	// Move cursor to home and clear
	fmt.Print(homePos)

	// Box drawing - all lines exactly 66 characters wide
	fmt.Println("╔════════════════════════════════════════════════════════════════╗")
	fmt.Println(makeLine("           MAVLink LoRa Receiver - Live Telemetry               "))
	fmt.Println("╠════════════════════════════════════════════════════════════════╣")
	fmt.Println(makeLine(" Port: /dev/cu.usbserial-0001                    Baud: 57600    "))
	fmt.Println("╠════════════════════════════════════════════════════════════════╣")

	// Counter and Messages
	content := fmt.Sprintf(" Counter:       %-10d                 Messages: %-8d ", counter, messageCount)
	fmt.Println(makeLine(content))

	// Last Update
	updateStr := lastUpdate.Format("2006-01-02 15:04:05")
	content = fmt.Sprintf(" Last Update:   %-47s ", updateStr)
	fmt.Println(makeLine(content))

	// Packets Lost
	if packetsLost > 0 {
		content = fmt.Sprintf(" Packets Lost:  %-46d ", packetsLost)
	} else {
		content = " Packets Lost:  None                                            "
	}
	fmt.Println(makeLine(content))

	fmt.Println("╠════════════════════════════════════════════════════════════════╣")
	fmt.Println(makeLine("                      SENSOR READINGS                           "))
	fmt.Println("╠════════════════════════════════════════════════════════════════╣")

	// Sensor readings with precise formatting
	content = fmt.Sprintf(" Temperature:   %-10.1f °C                                  ", temperature)
	fmt.Println(makeLine(content))

	content = fmt.Sprintf(" Humidity:      %-10.1f %%                                   ", humidity)
	fmt.Println(makeLine(content))

	content = fmt.Sprintf(" Pressure:      %-10.1f hPa                                 ", pressure)
	fmt.Println(makeLine(content))

	content = fmt.Sprintf(" Altitude:      %-10.1f m MSL                               ", altitude)
	fmt.Println(makeLine(content))

	fmt.Println("╚════════════════════════════════════════════════════════════════╝")
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
	var lastUpdate time.Time
	var packetsLost uint32

	// Draw initial table
	drawTable(counter, temperature, humidity, pressure, altitude, messageCount, time.Now(), packetsLost)

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
				drawTable(counter, temperature, humidity, pressure, altitude, messageCount, lastUpdate, packetsLost)
			}
		}
	}
}