# Complete Customization Guide

This guide explains every part of the simplified simulation and shows you exactly where to make changes.

---

## 📋 Table of Contents

1. [Understanding the Code Structure](#understanding-the-code-structure)
2. [Where to Change Parameters](#where-to-change-parameters)
3. [Understanding Each File](#understanding-each-file)
4. [Common Customizations](#common-customizations)
5. [Troubleshooting](#troubleshooting)

---

## 🏗️ Understanding the Code Structure

### The Flow (What Happens When You Run `simulate.py`)

```
1. Set Parameters (lines 35-59)
   ↓
2. Create Vehicle Object (lines 76-84)
   ↓
3. Create Parking Scenario (lines 87-88)
   ↓
4. Plan the Path (lines 98-108)
   ↓
5. Simulate Motion (lines 131-162)
   ↓
6. Save Outputs (lines 170-209)
```

---

## ⚙️ Where to Change Parameters

### Location: `simulate.py` (Lines 35-59)

**This is the MAIN place to customize everything!**

```python
# ============================================================================
# SIMULATION PARAMETERS (Hardcoded for simplicity)
# ============================================================================

# 🚗 STARTING POSITION - Change these to start from different locations
X_START = 75           # ← Change X starting position (meters)
Y_START = 40           # ← Change Y starting position (meters)
PSI_START = -20        # ← Change starting angle (degrees, -180 to 180)

# 🎯 TARGET POSITION - Usually auto-calculated, but you can adjust
X_END = 90             # ← Approximate target X (will be adjusted)
Y_END = 80             # ← Approximate target Y (will be adjusted)

# 🅿️ PARKING SPOT - Change these to test different parking scenarios
PARKING_LENGTH = 12    # ← Parking spot length (meters)
                       #   Smaller = tighter spot (harder to park)
                       #   Larger = spacious spot (easier to park)
PARKING_MARGIN = 1     # ← Safety margin around car (meters)

# 🚙 VEHICLE DIMENSIONS - Change these to simulate different vehicles
CAR_LENGTH = 8         # ← Car length (meters)
CAR_WIDTH = 4          # ← Car width (meters)
WHEEL_LENGTH = 1.5     # ← Wheel length (meters) - for visualization
WHEEL_WIDTH = 0.7      # ← Wheel width (meters) - for visualization

# Wheel positions relative to car center
# Format: [front_right, front_left, rear_right, rear_left]
WHEEL_POSITIONS = np.array([
    [2.0, 1.0],   # Front right wheel
    [2.0, -1.0],  # Front left wheel
    [-2.0, 1.0],  # Rear right wheel
    [-2.0, -1.0]  # Rear left wheel
])

# ⏱️ SIMULATION SPEED
DT = 0.2              # ← Time step (seconds)
                       #   Smaller = smoother but slower simulation
                       #   Larger = faster but less smooth
```

### Example Customizations:

**Test a tighter parking spot:**
```python
PARKING_LENGTH = 10  # Changed from 12 to 10 (tighter spot)
```

**Start from a different angle:**
```python
PSI_START = -30  # Changed from -20 to -30 degrees
```

**Simulate a smaller car:**
```python
CAR_LENGTH = 6   # Changed from 8 to 6 meters
CAR_WIDTH = 3    # Changed from 4 to 3 meters
```

---

## 📁 Understanding Each File

### 1. `simulate.py` - Main Script

**What it does:** Runs the entire simulation

**Key Sections:**

#### Section 1: Parameters (Lines 35-59)
- **What:** All customizable values
- **Change here:** Starting position, parking spot size, car dimensions

#### Section 2: Vehicle Initialization (Lines 76-84)
- **What:** Creates the car object with physics model
- **Don't change:** Usually fine as-is, uses parameters from Section 1

#### Section 3: Parking Scenario (Lines 87-88)
- **What:** Creates obstacles (other cars, walls)
- **Don't change:** Automatically creates scenario based on parking length

#### Section 4: Path Planning (Lines 98-108)
- **What:** Calculates the parking path
- **Don't change:** Automatically chooses best strategy

#### Section 5: Simulation Loop (Lines 131-162)
- **What:** Moves car along the path step-by-step
- **Don't change:** This is the core simulation logic

#### Section 6: Output Generation (Lines 170-209)
- **What:** Saves GIF and image files
- **Can change:** 
  - Line 190: `frame_skip = max(1, len(frames) // 150)` 
    - Change `150` to adjust GIF size (smaller number = more frames = larger file)
  - Line 195: `duration=0.1` 
    - Change to adjust GIF speed (smaller = faster animation)

---

### 2. `control.py` - Vehicle Physics

**What it does:** Defines how the car moves (physics equations)

**Key Parts:**

#### `Car_Dynamics` Class
- **What:** The car's physics model
- **Key Method:** `move()` - calculates how car moves based on steering/acceleration

**Where to customize:**

```python
# Line 90: Maximum steering angle
self.steer_max = np.deg2rad(40)  # ← Change 40 to allow more/less steering
                                  #   Larger = can turn sharper
                                  #   Smaller = wider turns only
```

**Example:** Allow sharper turns
```python
self.steer_max = np.deg2rad(50)  # Changed from 40 to 50 degrees
```

**Don't change:** The kinematic equations (lines 114-118) - these are physics!

---

### 3. `pathplanning.py` - Path Planning Algorithm

**What it does:** Calculates the optimal parking path

**Key Parts:**

#### `PathPlanning` Class
- **What:** Plans the geometric path using circular arcs

**Where to customize:**

```python
# Line 52: Point spacing for one-trial path
point_interval = 0.25  # ← Change for smoother/rougher path
                       #   Smaller = smoother path but more points
                       #   Larger = rougher path but fewer points

# Line 89: Point spacing for several-trial path  
point_interval = 0.1   # ← Same as above, for tight parking
```

**Example:** Smoother path
```python
point_interval = 0.15  # Changed from 0.25 to 0.15 (smoother)
```

**Don't change:** The geometric calculations - these are complex math!

---

### 4. `environment.py` - Visualization & Collision Detection

**What it does:** Draws everything and checks for collisions

**Key Parts:**

#### `Environment` Class
- **What:** Handles drawing and collision detection

**Where to customize:**

```python
# Line 17: Car color (RGB values 0-255)
self.color = np.array([0, 0, 255]) / 255  # ← Blue car
                                          #   Change to [255, 0, 0] for red
                                          #   Change to [0, 255, 0] for green

# Line 18: Wheel color
self.wheel_color = np.array([20, 20, 20]) / 255  # ← Dark gray wheels
```

**Example:** Red car
```python
self.color = np.array([255, 0, 0]) / 255  # Red car
```

**Don't change:** Collision detection logic - it's complex!

---

## 🎨 Common Customizations

### Customization 1: Change Parking Spot Size

**File:** `simulate.py`  
**Line:** 48

```python
PARKING_LENGTH = 10  # Smaller = tighter spot (harder)
PARKING_LENGTH = 15  # Larger = spacious spot (easier)
```

**What happens:**
- Smaller values → Uses "several-trial" path (multiple maneuvers)
- Larger values → Uses "one-trial" path (simple maneuver)

---

### Customization 2: Change Starting Position

**File:** `simulate.py`  
**Lines:** 39-41

```python
X_START = 80        # Move car further right
Y_START = 45        # Move car further up
PSI_START = -30     # Start at steeper angle
```

**What happens:**
- Different starting positions → Different parking paths
- Steeper angles → More complex maneuvers

---

### Customization 3: Change Car Size

**File:** `simulate.py`  
**Lines:** 52-53

```python
CAR_LENGTH = 6   # Smaller car
CAR_WIDTH = 3    # Narrower car
```

**What happens:**
- Smaller car → Easier to park in tight spots
- Larger car → Needs more space

---

### Customization 4: Change GIF Speed

**File:** `simulate.py`  
**Line:** 195

```python
imageio.mimwrite(gif_path, gif_frames, duration=0.05, loop=0)
#                                                      ^^^^
#                                                      Smaller = faster
```

**What happens:**
- `duration=0.05` → Faster animation
- `duration=0.2` → Slower animation

---

### Customization 5: Change Car Color

**File:** `environment.py`  
**Line:** 17

```python
self.color = np.array([255, 0, 0]) / 255  # Red
self.color = np.array([0, 255, 0]) / 255  # Green
self.color = np.array([0, 0, 255]) / 255  # Blue (default)
```

---

## 🔍 Understanding Confusing Parts

### Part 1: "Rear Axle vs Vehicle Center"

**Why it's confusing:** The path is planned using rear axle positions, but we visualize using vehicle center.

**What it means:**
- **Rear axle:** The back wheels (where path planning happens)
- **Vehicle center:** Middle of the car (where visualization happens)

**Where it's used:**
```python
# Line 101-102: Convert vehicle center to rear axle for planning
start_rear_x = start[0] - my_car.a / 2 * np.cos(np.deg2rad(PSI_START))

# Line 147-148: Convert rear axle back to vehicle center for visualization
x_center = x + my_car.a / 2 * np.cos(psi)
```

**Don't worry about it:** The code handles this automatically!

---

### Part 2: "One-Trial vs Several-Trial"

**Why it's confusing:** The code automatically chooses between two strategies.

**What it means:**
- **One-trial:** Simple path (right turn, then left turn) - for spacious spots
- **Several-trial:** Multiple maneuvers (back-and-forth) - for tight spots

**Where it's decided:**
```python
# Line 26-32 in pathplanning.py
if self.parking.parking_length > self.L_min + ...:
    path, steer = self.plan_path_one_trial(...)  # Spacious spot
else:
    path, steer = self.plan_path_several_trial(...)  # Tight spot
```

**How to control it:**
- Make `PARKING_LENGTH` larger → Forces one-trial
- Make `PARKING_LENGTH` smaller → Forces several-trial

---

### Part 3: "Steering Angles"

**Why it's confusing:** Steering angles can be positive or negative.

**What it means:**
- **Positive angle:** Turn left
- **Negative angle:** Turn right
- **Zero:** Go straight

**Where it's used:**
```python
# Line 133: Get steering angle for current waypoint
delta = steer_angles[i]  # Could be positive (left) or negative (right)
```

**Don't worry:** The path planner calculates these automatically!

---

### Part 4: "Waypoints"

**Why it's confusing:** The path is made of discrete points, not continuous.

**What it means:**
- **Waypoint:** A point on the path
- **Path:** Collection of waypoints
- Car moves from waypoint to waypoint

**Where it's used:**
```python
# Line 131: Loop through each waypoint
for i in range(len(path)):
    point = path[i]  # Current waypoint
    # Move car to this waypoint
```

**To make smoother:** Reduce `point_interval` in `pathplanning.py`

---

## 🐛 Troubleshooting

### Problem: Car doesn't park correctly

**Solution:**
1. Check `PARKING_LENGTH` - might be too small
2. Check starting position - might be too far
3. Check `PSI_START` - might be wrong angle

### Problem: GIF is too large/small

**Solution:**
1. Change `frame_skip` calculation (line 190)
2. Change `duration` (line 195)

### Problem: Car collides with obstacles

**Solution:**
1. Increase `PARKING_MARGIN` (line 49)
2. Increase `PARKING_LENGTH` (line 48)
3. Check if starting position is valid

### Problem: Path looks choppy

**Solution:**
1. Reduce `point_interval` in `pathplanning.py` (lines 52, 89)
2. Reduce `DT` in `simulate.py` (line 59)

---

## 📝 Quick Reference: All Customization Points

| What to Change | File | Line | Example |
|---------------|------|------|---------|
| Starting X position | `simulate.py` | 39 | `X_START = 80` |
| Starting Y position | `simulate.py` | 40 | `Y_START = 45` |
| Starting angle | `simulate.py` | 41 | `PSI_START = -30` |
| Parking spot size | `simulate.py` | 48 | `PARKING_LENGTH = 10` |
| Safety margin | `simulate.py` | 49 | `PARKING_MARGIN = 1.5` |
| Car length | `simulate.py` | 52 | `CAR_LENGTH = 6` |
| Car width | `simulate.py` | 53 | `CAR_WIDTH = 3` |
| Time step | `simulate.py` | 59 | `DT = 0.1` |
| Max steering angle | `control.py` | 90 | `steer_max = 50` |
| Path smoothness | `pathplanning.py` | 52, 89 | `point_interval = 0.15` |
| Car color | `environment.py` | 17 | `[255, 0, 0]` (red) |
| GIF speed | `simulate.py` | 195 | `duration=0.05` |
| GIF file size | `simulate.py` | 190 | `// 200` (more frames) |

---

## 💡 Tips

1. **Start simple:** Change one parameter at a time
2. **Test incrementally:** Run simulation after each change
3. **Check outputs:** Look at the GIF to see if changes worked
4. **Read error messages:** They usually tell you what's wrong
5. **Backup before changes:** Copy files before major modifications

---

## 🎓 For Case Study Presentation

**What to explain:**
1. **Parameters section** (lines 35-59): "These control everything"
2. **Path planning** (lines 98-108): "This calculates the path automatically"
3. **Simulation loop** (lines 131-162): "This moves the car step-by-step"
4. **Outputs** (lines 170-209): "This creates the visualizations"

**What NOT to explain:**
- Complex geometric calculations
- Coordinate transformations
- Collision detection algorithms

**Focus on:**
- The parameters you can change
- The automatic path planning
- The visual results

---

Need more help? Check the code comments - they explain what each section does!

