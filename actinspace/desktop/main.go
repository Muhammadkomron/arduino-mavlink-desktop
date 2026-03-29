package main

import (
	"context"
	"embed"
	"os"

	"cansat-ground-station/backend"

	"github.com/wailsapp/wails/v2"
	"github.com/wailsapp/wails/v2/pkg/options"
	"github.com/wailsapp/wails/v2/pkg/options/assetserver"
	"github.com/wailsapp/wails/v2/pkg/options/mac"
)

//go:embed all:frontend/dist
var assets embed.FS

func main() {
	app := backend.NewApp()

	// Check if this is a standalone view child window
	viewMode := os.Getenv("CANSAT_VIEW")

	if viewMode != "" {
		// --- Child window: show only the standalone view ---
		title := "NazarX - " + viewTitle(viewMode)

		err := wails.Run(&options.App{
			Title:            title,
			Width:            1200,
			Height:           800,
			MinWidth:         800,
			MinHeight:        600,
			DisableResize:    false,
			WindowStartState: options.Fullscreen,
			Mac: &mac.Options{
				TitleBar: mac.TitleBarDefault(),
				Preferences: &mac.Preferences{
					FullscreenEnabled: mac.Enabled,
				},
			},
			AssetServer: &assetserver.Options{
				Assets: assets,
			},
			BackgroundColour: &options.RGBA{R: 15, G: 15, B: 26, A: 1},
			OnStartup:        app.StartupChild,
			OnShutdown:       func(_ context.Context) {},
			Bind: []interface{}{
				app,
			},
		})
		if err != nil {
			println("Error:", err.Error())
		}
		return
	}

	// --- Main dashboard window ---
	err := wails.Run(&options.App{
		Title:            "NazarX Ground Station",
		Width:            1440,
		Height:           900,
		MinWidth:         1200,
		MinHeight:        700,
		DisableResize:    false,
		WindowStartState: options.Fullscreen,
		Mac: &mac.Options{
			TitleBar: mac.TitleBarDefault(),
			Preferences: &mac.Preferences{
				FullscreenEnabled: mac.Enabled,
			},
		},
		AssetServer: &assetserver.Options{
			Assets: assets,
		},
		BackgroundColour: &options.RGBA{R: 15, G: 15, B: 26, A: 1},
		OnStartup:        app.Startup,
		OnShutdown:       app.Shutdown,
		Bind: []interface{}{
			app,
		},
	})

	if err != nil {
		println("Error:", err.Error())
	}
}

func viewTitle(view string) string {
	switch view {
	case "gps":
		return "GPS Tracking"
	case "3d":
		return "3D Orientation"
	case "video":
		return "Video Feed"
	default:
		return view
	}
}
