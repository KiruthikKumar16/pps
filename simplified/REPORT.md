# Task 1.1 (Simplified): Kinematic Simulation of a Self-Driving Car Parallel Parking Maneuver

## Executive Summary

This report documents the **simplified** kinematic simulation prototype for autonomous parallel parking. It keeps the core elements—bicycle vehicle model, geometric trajectory generation with circular arcs, collision checking, and visual outputs—while removing advanced options (MPC, CLI arguments, extensive plots) to make the case study easier to explain and run. The simulation demonstrates collision-free parking with visual validation via an animated GIF and a final trajectory image.

---

## 1. Kinematic Model Description

### 1.1 Vehicle Model
The vehicle uses a **kinematic bicycle model** (single-track). It captures nonholonomic motion with minimal complexity—ideal for low-speed parking.

#### 1.1.1 Vehicle Dimensions (default)
- Car Length: 8.0 m
- Car Width: 4.0 m
- Wheelbase (L): 4.0 m (from wheel positions)
- Wheel Length: 1.5 m
- Wheel Width: 0.7 m
- Max Steering Angle: ±40°

#### 1.1.2 State Variables
```
x = [x, y, v, ψ]ᵀ
```
- x, y: position (m)
- v: speed (m/s)
- ψ: heading (rad)

#### 1.1.3 Inputs
```
u = [a, δ]ᵀ
```
- a: acceleration (m/s²)
- δ: steering (rad)

### 1.2 Kinematic Equations
```
ẋ = v · cos(ψ)
ẏ = v · sin(ψ)
v̇ = a
ψ̇ = (v · tan(δ)) / L
```
Implemented in `Car_Dynamics.move()`:
```
x_dot = self.v * np.cos(self.psi)
y_dot = self.v * np.sin(self.psi)
v_dot = accelerate
psi_dot = self.v * np.tan(delta) / self.L
```

### 1.3 Nonholonomic Constraints
- Vehicle cannot move sideways; heading governs motion.
- Minimum turning radius: `R_min = L / tan(δ_max)`.

### 1.4 State Update
Euler integration with `dt = 0.2 s`:
```
x(k+1) = x(k) + dt · ẋ(k)
```

---

## 2. Trajectory Planning

### 2.1 Geometric Path Planning
Uses circular arcs and straight segments with vehicle constraints:
- `R_Elmin`: min left-turn radius
- `R_Ermin`: min right-turn radius
- `R_Bl_min`: min backward-left radius
- `L_min`: min parking length

### 2.2 Path Generation Strategy
Two strategies (auto-selected):
1) **One-trial path** (spacious spots): right turn then left turn.
2) **Several-trial path** (tight spots): alternating left/right maneuvers.

Waypoints are spaced 0.1–0.25 m; steering angles computed per waypoint.

---

## 3. Simulation (Simplified Run)

### 3.1 Parameters (hardcoded in `simulate.py`)
- Start: (75, 40) m
- Start Heading: -20°
- Parking Length: 12 m
- Parking Margin: 1 m
- dt: 0.2 s

### 3.2 Execution Flow
1. Initialize car model.
2. Build parking scenario and obstacles.
3. Plan path (auto choose one- vs several-trial).
4. Follow path in open-loop (no MPC) and render each step.
5. Collision check each frame.
6. Save outputs.

### 3.3 Outputs
- `reports/parking_maneuver.gif` (animated maneuver)
- `reports/reference_trajectory.png` (final trajectory with footprints)

---

## 4. Results (Example Run)
- Final position shown in `reference_trajectory.png`.
- No collisions observed in the provided scenario.
- Visual validation via GIF confirms successful parking.
- Open-loop following; minor error acceptable for demo.

---

## 5. Challenges & Simplified Solutions

1) **Nonholonomic motion**  
   - Used bicycle kinematics with turning-radius constraints.

2) **Tight-spot geometry**  
   - Provided both one-trial and several-trial planners; auto-select based on spot length.

3) **Frame alignment (rear axle vs. vehicle center)**  
   - Applied offset `x_center = x_base + (a/2) cos(ψ)` and `y_center = y_base + (a/2) sin(ψ)`.

4) **Collision checking**  
   - Pixel-mask collision using OpenCV polygons each step.

5) **Complexity reduction for case study**  
   - Removed MPC, CLI args, and time-series plots; kept visuals (GIF + trajectory).

---

## 6. Code Structure (Simplified)
- `simulate.py` — main scripted run (hardcoded params, open-loop following)
- `control.py` — kinematic bicycle model
- `pathplanning.py` — geometric planner (one-trial & several-trial)
- `environment.py` — visualization + collision detection
- `requirements.txt` — minimal deps (numpy, opencv-python, imageio/Pillow)

---

## 7. Conclusions
### Key Achievements
1. ✅ Geometric path planning for parallel parking (two strategies)
2. ✅ Kinematic bicycle simulation with collision checking
3. ✅ Visual evidence of successful parking (GIF + trajectory)
4. ✅ Reduced complexity for clear case-study storytelling

### Recommendations
1. Add MPC back for tighter tracking if needed.
2. Reintroduce time-series plots for deeper analysis.
3. Parameterize via CLI if multiple scenarios are desired.
4. Explore dynamic obstacles and perception modules as next steps.

---

## Appendix A: Mathematical Formulas
### A.1 Bicycle Model
```
ẋ = v · cos(ψ)
ẏ = v · sin(ψ)
v̇ = a
ψ̇ = (v · tan(δ)) / L
```

### A.2 Minimum Turning Radius
```
R_min = L / tan(δ_max)
```

---

**Version:** Simplified 1.0  
**Scope:** Educational / case-study demonstration  

