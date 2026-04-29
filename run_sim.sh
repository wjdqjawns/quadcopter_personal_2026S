#!/bin/bash
# Quick simulation launcher script

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$PROJECT_DIR/build"
SIM_BIN="$BUILD_DIR/drone_sim"

# Check if binary exists
if [ ! -f "$SIM_BIN" ]; then
    echo "❌ Binary not found: $SIM_BIN"
    echo "Building first..."
    cd "$BUILD_DIR"
    cmake ..
    make -j$(nproc)
    echo "✓ Build complete"
fi

cd "$PROJECT_DIR"

# Check for display
if [ -z "$DISPLAY" ]; then
    echo "⚠️  Warning: No X11 display detected"
    echo ""
    echo "Options:"
    echo "  1) X11 Forwarding Setup:"
    echo "     export DISPLAY=:0"
    echo "     $SIM_BIN"
    echo ""
    echo "  2) Run Headless (no visualization):"
    echo "     Edit config/sim/skydio_x2.yaml"
    echo "     Change: visualize: true"
    echo "     To:     visualize: false"
    echo ""
    echo "  3) Use VNC/Remote Display"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 0
    fi
fi

echo "🚀 Starting Drone Simulation..."
echo "Config: config/sim/skydio_x2.yaml"
echo "Output log will be saved to: data/logs/sim_log.csv"
echo ""
echo "Controls:"
echo "  • Right-Click + Drag: Rotate camera"
echo "  • Left-Click + Drag: Pan camera"
echo "  • Scroll: Zoom in/out"
echo "  • ESC: Close window"
echo ""

exec "$SIM_BIN" "${1:-config/sim/skydio_x2.yaml}"
