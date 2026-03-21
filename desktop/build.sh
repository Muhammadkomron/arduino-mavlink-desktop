#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

APP_NAME="NazarX-Ground-Station"
OUTPUT_DIR="build/bin"

echo "========================================"
echo "  NazarX Ground Station - Build"
echo "========================================"
echo ""

# Check wails is installed
if ! command -v wails &>/dev/null; then
    echo "[ERROR] Wails CLI not found. Install it with:"
    echo "  go install github.com/wailsapp/wails/v2/cmd/wails@latest"
    exit 1
fi

# Parse arguments
PLATFORM="${1:-all}"
SKIP_FRONTEND="${SKIP_FRONTEND:-false}"

show_usage() {
    echo "Usage: ./build.sh [platform]"
    echo ""
    echo "Platforms:"
    echo "  mac         Build for macOS (current arch)"
    echo "  mac-amd64   Build for macOS Intel"
    echo "  mac-arm64   Build for macOS Apple Silicon"
    echo "  mac-universal  Build for macOS Universal"
    echo "  windows     Build for Windows amd64"
    echo "  linux       Build for Linux amd64"
    echo "  all         Build for all platforms (default)"
    echo ""
    echo "Environment variables:"
    echo "  SKIP_FRONTEND=true  Skip frontend build (use existing dist)"
    echo ""
    echo "Examples:"
    echo "  ./build.sh mac"
    echo "  ./build.sh windows"
    echo "  SKIP_FRONTEND=true ./build.sh all"
}

build_frontend() {
    if [ "$SKIP_FRONTEND" = "true" ]; then
        echo "[SKIP] Using existing frontend build"
        return
    fi
    echo "--- Building frontend ---"
    cd frontend
    npm install --silent
    npm run build
    cd "$SCRIPT_DIR"
    echo "[DONE] Frontend built"
    echo ""
}

build_mac() {
    local arch="${1:-$(uname -m)}"
    local goarch=""

    case "$arch" in
        arm64|aarch64) goarch="arm64" ;;
        amd64|x86_64) goarch="amd64" ;;
        universal) goarch="universal" ;;
        *) echo "[ERROR] Unknown arch: $arch"; return 1 ;;
    esac

    echo "--- Building macOS ($goarch) ---"

    if [ "$goarch" = "universal" ]; then
        wails build -platform darwin/universal -clean
    else
        wails build -platform "darwin/$goarch" -clean
    fi

    # Create DMG
    local app_path="$OUTPUT_DIR/${APP_NAME}.app"
    local dmg_path="$OUTPUT_DIR/${APP_NAME}-macOS-${goarch}.dmg"

    if [ -d "$app_path" ]; then
        # Ad-hoc sign
        codesign --force --deep --sign - "$app_path" 2>/dev/null || true

        # Create DMG
        rm -f "$dmg_path"
        hdiutil create -volname "NazarX Ground Station" \
            -srcfolder "$app_path" \
            -ov -format UDZO \
            "$dmg_path" 2>/dev/null

        echo "[DONE] macOS ($goarch): $dmg_path"
    else
        echo "[DONE] macOS ($goarch): $app_path"
    fi
    echo ""
}

build_windows() {
    echo "--- Building Windows (amd64) ---"

    if [[ "$(uname)" == "Darwin" || "$(uname)" == "Linux" ]]; then
        # Cross-compile from macOS/Linux
        GOOS=windows GOARCH=amd64 CGO_ENABLED=1 wails build -platform windows/amd64 -clean -nsis 2>/dev/null || \
        GOOS=windows GOARCH=amd64 wails build -platform windows/amd64 -clean 2>/dev/null || {
            echo "[WARN] Cross-compiling for Windows requires:"
            echo "  macOS: brew install mingw-w64"
            echo "  Linux: apt install gcc-mingw-w64"
            echo "[SKIP] Windows build skipped (missing cross-compiler)"
            return 0
        }
    else
        wails build -platform windows/amd64 -clean
    fi

    echo "[DONE] Windows (amd64): $OUTPUT_DIR/${APP_NAME}.exe"
    echo ""
}

build_linux() {
    echo "--- Building Linux (amd64) ---"

    if [[ "$(uname)" == "Darwin" ]]; then
        echo "[WARN] Cross-compiling for Linux from macOS requires Docker."
        echo "  Consider building on a Linux machine or using Docker:"
        echo "  docker run --rm -v \$(pwd):/app -w /app ghcr.io/nicholasgasior/wails-docker:latest wails build"
        echo "[SKIP] Linux build skipped"
        return 0
    fi

    if [[ "$(uname)" == "Linux" ]]; then
        # Check for required Linux deps
        if ! pkg-config --exists gtk+-3.0 webkit2gtk-4.0 2>/dev/null; then
            echo "[WARN] Missing Linux dependencies. Install with:"
            echo "  sudo apt install libgtk-3-dev libwebkit2gtk-4.0-dev"
            echo "[SKIP] Linux build skipped"
            return 0
        fi
        wails build -platform linux/amd64 -clean
        echo "[DONE] Linux (amd64): $OUTPUT_DIR/${APP_NAME}"
    fi
    echo ""
}

# Build frontend once
build_frontend

case "$PLATFORM" in
    mac)
        build_mac
        ;;
    mac-amd64)
        build_mac amd64
        ;;
    mac-arm64)
        build_mac arm64
        ;;
    mac-universal)
        build_mac universal
        ;;
    windows)
        build_windows
        ;;
    linux)
        build_linux
        ;;
    all)
        build_mac
        build_windows
        build_linux
        ;;
    help|-h|--help)
        show_usage
        exit 0
        ;;
    *)
        echo "[ERROR] Unknown platform: $PLATFORM"
        echo ""
        show_usage
        exit 1
        ;;
esac

echo "========================================"
echo "  Build complete!"
echo "  Output: $OUTPUT_DIR/"
echo "========================================"

ls -lh "$OUTPUT_DIR/" 2>/dev/null || true
