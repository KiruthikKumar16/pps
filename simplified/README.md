# Simplified Parallel Parking Simulation

A simplified, easy-to-understand version of the parallel parking simulation project. This version focuses on the core concepts while removing advanced features like MPC control and complex plotting.

## 🎯 What This Does

This simulation demonstrates:
1. **Geometric Path Planning**: Calculates the optimal parking path using circular arcs
2. **Kinematic Vehicle Model**: Simulates car motion using physics equations
3. **Collision Detection**: Ensures the car doesn't hit obstacles
4. **Visualization**: Creates animated GIF and trajectory plots

## 🚀 Quick Start

### Installation

```bash
pip install -r requirements.txt
```

### Run Simulation

```bash
python simulate.py
```

That's it! The simulation will:
- Plan the parking path automatically
- Simulate the vehicle following the path
- Generate `reports/parking_maneuver.gif` (animated parking)
- Generate `reports/reference_trajectory.png` (final trajectory)

## 📁 Project Structure

```
simplified/
├── simulate.py          # Main simulation script (hardcoded parameters)
├── control.py           # Vehicle dynamics model (kinematic bicycle model)
├── pathplanning.py      # Geometric path planning using circular arcs
├── environment.py       # Visualization and collision detection
├── requirements.txt     # Python dependencies
└── README.md           # This file
```

## 🔑 Key Concepts

### 1. Kinematic Bicycle Model

The car's motion is modeled using these equations:

```
ẋ = v · cos(ψ)          # x-velocity
ẏ = v · sin(ψ)          # y-velocity  
v̇ = a                   # acceleration
ψ̇ = (v · tan(δ)) / L    # yaw rate (turning)
```

Where:
- `x, y`: Position coordinates
- `v`: Linear velocity
- `ψ`: Heading angle
- `a`: Acceleration input
- `δ`: Steering angle input
- `L`: Wheelbase

### 2. Path Planning Strategy

The planner automatically chooses:
- **One-trial path**: Simple right-then-left turn (for spacious spots)
- **Several-trial path**: Multiple back-and-forth maneuvers (for tight spots)

Both use circular arcs based on the vehicle's minimum turning radius.

### 3. Simulation Flow

1. Initialize vehicle and parking scenario
2. Plan geometric path
3. Simulate vehicle following path step-by-step
4. Check for collisions at each step
5. Generate visualization outputs

## 📊 Output Files

After running `simulate.py`:

- **`reports/parking_maneuver.gif`**: Animated GIF showing the entire parking maneuver
- **`reports/reference_trajectory.png`**: Static image showing the final trajectory with vehicle footprints

## ⚙️ Configuration

All parameters are hardcoded at the top of `simulate.py` for simplicity:

```python
X_START = 75           # Starting X position
Y_START = 40           # Starting Y position
PSI_START = -20        # Starting heading (degrees)
PARKING_LENGTH = 12    # Parking spot length (meters)
CAR_LENGTH = 8         # Vehicle length (meters)
CAR_WIDTH = 4          # Vehicle width (meters)
```

Modify these values to test different scenarios!

## 🎓 For Case Study Presentation

This simplified version is perfect for case studies because:

1. **Easy to Explain**: Clear, well-commented code
2. **Focused**: Only essential features, no distractions
3. **Complete**: Still demonstrates all core concepts
4. **Visual**: Produces clear visualizations
5. **Self-contained**: Single command to run everything

### Presentation Flow:

1. **Problem**: "How can an autonomous car park itself?"
2. **Approach**: 
   - Geometric path planning (circular arcs)
   - Kinematic model (physics simulation)
   - Collision checking
3. **Implementation**: Show the code structure
4. **Results**: Show the GIF and trajectory image
5. **Conclusion**: "Successfully demonstrates autonomous parking"

## 🔄 Differences from Full Version

This simplified version:
- ✅ Removed MPC controller (uses open-loop path following)
- ✅ Removed time-series plots (only GIF and trajectory)
- ✅ Hardcoded parameters (no command-line arguments)
- ✅ Simplified code structure
- ✅ Added extensive comments

The full version includes:
- MPC control for trajectory tracking
- Multiple time-series plots
- Command-line argument parsing
- Interactive mode

## 📚 Technical Details

### Vehicle Parameters
- Length: 8m
- Width: 4m
- Maximum steering angle: ±40°
- Wheelbase: 4m

### Path Planning
- Uses geometric calculations (Al-Kashi's theorem)
- Considers vehicle constraints (turning radius, dimensions)
- Automatically selects optimal strategy

### Collision Detection
- Checks vehicle footprint against obstacles at each step
- Uses pixel-level collision detection for accuracy

## 🤝 Contributing

This is a simplified educational version. For the full-featured version, see the parent directory.

## 📝 License

Educational use only.

