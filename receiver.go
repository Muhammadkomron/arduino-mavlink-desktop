package main

import (
	"fmt"
	"log"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
)

func main() {
	fmt.Println("==============================================")
	fmt.Println("MAVLink LoRa Receiver")
	fmt.Println("Port: /dev/cu.usbserial-0001")
	fmt.Println("Baud: 57600")
	fmt.Println("==============================================\n")

	// Create MAVLink node
	node, err := gomavlib.NewNode(gomavlib.NodeConf{
		Endpoints: []gomavlib.EndpointConf{
			gomavlib.EndpointSerial{
				Device: "/dev/cu.usbserial-0001",  // Receiver LoRa USB
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

	fmt.Println("Listening for MAVLink messages...")
	fmt.Println("Press Ctrl+C to exit\n")

	messageCount := 0
	var lastCounter uint32
	var temperature float32
	var pressure float32
	var altitude float32

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
				timestamp := time.Now().Format("15:04:05")

				fmt.Printf("[%s] BMP280: %.1f°C  %.1f hPa  %.1f m\n",
					timestamp, temperature, pressure, altitude)

			case *common.MessageVfrHud:
				// ArduPilot VFR HUD with altitude
				altitude = msg.Alt
				timestamp := time.Now().Format("15:04:05")

				fmt.Printf("[%s] Altitude: %.1f m MSL\n",
					timestamp, altitude)

			case *common.MessageHeartbeat:
				// Heartbeat with counter
				messageCount++
				counter := msg.CustomMode
				timestamp := time.Now().Format("15:04:05")

				fmt.Printf("[%s] Heartbeat #%d: Count=%d",
					timestamp, messageCount, counter)

				// Check for missed packets
				if messageCount > 1 && counter != lastCounter+1 {
					missed := counter - lastCounter - 1
					fmt.Printf(" ⚠ missed %d", missed)
				}
				fmt.Println()

				lastCounter = counter
			}
		}
	}
}