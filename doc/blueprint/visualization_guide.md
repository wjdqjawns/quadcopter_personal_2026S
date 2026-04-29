# Drone Simulation with Live Visualization

## What Was Updated

Your drone simulation now includes **live state visualization with real-time data display**:

### Enhanced Features

1. **Live 3D View** - MuJoCo 3D visualization with automatic drone tracking
2. **Real-Time State Overlays** - Four corner displays showing:
   - **Top-Left**: Ground truth pose (position, velocity, orientation)
   - **Top-Right**: EKF estimation (estimated position/velocity & complementary filter attitude)
   - **Bottom-Left**: Target position & tracking error
   - **Bottom-Right**: Control outputs (thrust, torques, motor forces) + motor range history

3. **History Tracking** - Automatic tracking of min/max values for:
   - Position ranges
   - Motor force ranges
   - Helps assess trajectory limits

### New Code Components

- **PlotHistory struct**: Maintains deques of historical state values (1000-frame buffer)
- **renderTextOverlays()**: Renders all MuJoCo text overlays with current + historical stats
- **Enhanced main loop**: Calls `history_.push(d)` each frame to track values

## How to Run

### Prerequisites
```bash
# Ensure you have X11 or a display server
export DISPLAY=:0  # Adjust to your display

# Or use WSL2 with X11 forwarding from Windows
# Or use VNC for remote displays
```

### Basic Usage
```bash
cd /home/cdj-desk/Documents/prj_drone
./build/drone_sim
```

**Optional**: Specify custom config file
```bash
./build/drone_sim config/sim/skydio_x2.yaml
```

### Running Headless (No Display)

If you don't have a display server, set this environment variable to disable visualization:

```bash
# Edit config/sim/skydio_x2.yaml
# Change: visualize: true
# To:     visualize: false
```

Then run:
```bash
./build/drone_sim
```

The simulation will run silently and save data to `data/logs/sim_log.csv`.

## Display Controls

Once the 3D window opens:

- **Mouse Right-Click + Drag**: Rotate camera around drone
- **Mouse Left-Click + Drag**: Pan camera viewpoint  
- **Scroll Wheel**: Zoom in/out
- **ESC**: Close simulation window

## Output Files

After simulation runs:
- **CSV Log**: `data/logs/sim_log.csv` - Complete trajectory data (time, position, velocity, orientation, motor commands)
- **Analysis Scripts**: 
  - `scripts/plot_trajectory.py` - Visualize trajectory path
  - `scripts/plot_kalman.py` - Show estimation vs ground truth
  - `scripts/analyze_log.py` - Print statistics

### View Results
```bash
python3 scripts/plot_trajectory.py
python3 scripts/plot_kalman.py
```

## Display Components Layout

```
┌─────────────────────────────────────────────┐
│  [TOP-LEFT]          [3D VIEW]   [TOP-RIGHT]│
│  Ground Truth         3D Drone      EKF Est │
│  Pose & Velocity     Tracking        & CFAtt│
├─────────────────────────────────────────────┤
│ [BOTTOM-LEFT]       [3D VIEW]  [BOTTOM-RT] │
│ Target Pos          Drone           Control│
│ Tracking Error      Motion          Outputs│
└─────────────────────────────────────────────┘
```

## Customization

### Modify Display Information

Edit `sim/visualizer.cpp`, function `renderTextOverlays()` to change what's shown in each corner:

```cpp
// Example: Add custom overlay at specific location
char custom_title[256], custom_val[256];
std::snprintf(custom_title, ...);
std::snprintf(custom_val, ...);
mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, vp, custom_title, custom_val, &con_);
```

### Change Display Resolution

```bash
# Edit src/main.cpp line with visualizer creation:
vis = std::make_unique<sim::Visualizer>(env, 1920, 1080);  // Custom size
```

### Adjust Camera Tracking

Edit `sim/visualizer.cpp` in `Visualizer::Visualizer()`:

```cpp
cam_.distance   = 2.0;      // Distance from drone
cam_.elevation  = -20.0;    // View angle (°)
cam_.azimuth    = 45.0;     // Rotation (°)
```

## Troubleshooting

**Problem**: "Cannot load window" or "glfwInit failed"
- **Solution**: Ensure you have X11 forwarding or VNC set up properly

**Problem**: Slow rendering
- **Solution**: Run simulation headless to focus on computation, then analyze logs

**Problem**: Want to see only data without 3D visualization
- **Solution**: Set `visualize: false` in config file

## Technical Details

- **Update Rate**: 100 Hz (0.01s timestep from x2.xml)
- **History Buffer**: Last 1000 frames (~10 seconds at 100 Hz)
- **Overlay Font**: MuJoCo standard rendering with 150% scale
- **Camera**: Automatic tracking of "x2" body from scene

## Next Steps

1. **For Analysis**: Use Python scripts to process CSV logs
2. **For Tuning**: Modify PID gains in `config/controller_params.yaml`
3. **For Testing**: Change trajectory in `src/main.cpp` (waypoints, circle, etc.)
