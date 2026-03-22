package backend

import (
	"encoding/json"
	"fmt"
	"net/http"
	"sync"
)

const ssePort = 9877

type sseServer struct {
	mu           sync.RWMutex
	clients      map[chan []byte]struct{}
	themeMu      sync.RWMutex
	themeClients map[chan string]struct{}
}

var sse = &sseServer{
	clients:      make(map[chan []byte]struct{}),
	themeClients: make(map[chan string]struct{}),
}

// StartSSEServer starts a lightweight HTTP server for telemetry streaming to browser windows
func StartSSEServer() {
	mux := http.NewServeMux()

	mux.HandleFunc("/telemetry", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/event-stream")
		w.Header().Set("Cache-Control", "no-cache")
		w.Header().Set("Connection", "keep-alive")
		w.Header().Set("Access-Control-Allow-Origin", "*")

		flusher, ok := w.(http.Flusher)
		if !ok {
			http.Error(w, "streaming not supported", http.StatusInternalServerError)
			return
		}

		ch := make(chan []byte, 16)
		sse.mu.Lock()
		sse.clients[ch] = struct{}{}
		sse.mu.Unlock()

		defer func() {
			sse.mu.Lock()
			delete(sse.clients, ch)
			sse.mu.Unlock()
		}()

		ctx := r.Context()
		for {
			select {
			case <-ctx.Done():
				return
			case data := <-ch:
				fmt.Fprintf(w, "data: %s\n\n", data)
				flusher.Flush()
			}
		}
	})

	// Theme SSE endpoint — child windows subscribe to theme changes
	mux.HandleFunc("/theme", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/event-stream")
		w.Header().Set("Cache-Control", "no-cache")
		w.Header().Set("Connection", "keep-alive")
		w.Header().Set("Access-Control-Allow-Origin", "*")

		flusher, ok := w.(http.Flusher)
		if !ok {
			http.Error(w, "streaming not supported", http.StatusInternalServerError)
			return
		}

		ch := make(chan string, 4)
		sse.themeMu.Lock()
		sse.themeClients[ch] = struct{}{}
		sse.themeMu.Unlock()

		defer func() {
			sse.themeMu.Lock()
			delete(sse.themeClients, ch)
			sse.themeMu.Unlock()
		}()

		ctx := r.Context()
		for {
			select {
			case <-ctx.Done():
				return
			case theme := <-ch:
				fmt.Fprintf(w, "data: %s\n\n", theme)
				flusher.Flush()
			}
		}
	})

	// CORS preflight
	mux.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Access-Control-Allow-Origin", "*")
		w.WriteHeader(200)
	})

	go func() {
		if err := http.ListenAndServe(fmt.Sprintf(":%d", ssePort), mux); err != nil {
			// Port already in use (main process has it) — silently ignore
			fmt.Printf("SSE server: %v (OK if child window)\n", err)
		}
	}()
}

// BroadcastTheme sends a theme change to all connected child windows
func BroadcastTheme(theme string) {
	sse.themeMu.RLock()
	defer sse.themeMu.RUnlock()

	for ch := range sse.themeClients {
		select {
		case ch <- theme:
		default:
		}
	}
}

// BroadcastTelemetry sends telemetry data to all connected SSE clients
func BroadcastTelemetry(t TelemetryData) {
	data, err := json.Marshal(t)
	if err != nil {
		return
	}

	sse.mu.RLock()
	defer sse.mu.RUnlock()

	for ch := range sse.clients {
		select {
		case ch <- data:
		default:
			// drop if client is slow
		}
	}
}
