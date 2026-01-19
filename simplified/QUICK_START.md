# Quick Start Guide - Understanding the Code

## 🎯 The Simplest Explanation

**What this code does:**
1. Creates a car
2. Plans a parking path
3. Moves the car along that path
4. Makes a GIF showing the parking

That's it!

---

## 📍 Where Everything Is

### Main File: `simulate.py`

This is the ONLY file you need to understand for basic customization!

```
simulate.py
├── Lines 35-59:  ⚙️ PARAMETERS (Change these!)
├── Lines 76-84:  🚗 Create car
├── Lines 87-88:  🅿️ Create parking spot
├── Lines 98-108: 📐 Plan path
├── Lines 131-162: ▶️ Simulate motion
└── Lines 170-209: 💾 Save outputs
```

---

## 🔧 The 5 Most Important Parameters

All in `simulate.py` lines 35-59:

### 1. Starting Position
```python
X_START = 75      # Where car starts (left-right)
Y_START = 40      # Where car starts (up-down)
PSI_START = -20   # What angle car faces (degrees)
```

### 2. Parking Spot Size
```python
PARKING_LENGTH = 12  # How big the parking spot is
PARKING_MARGIN = 1   # Safety space around car
```

### 3. Car Size
```python
CAR_LENGTH = 8   # How long the car is
CAR_WIDTH = 4    # How wide the car is
```

### 4. Simulation Speed
```python
DT = 0.2  # Time between steps (smaller = smoother)
```

---

## 🎨 What Each File Does

| File | What It Does | Should You Change It? |
|------|--------------|----------------------|
| `simulate.py` | Main script - runs everything | ✅ YES - Change parameters |
| `control.py` | Car physics (how car moves) | ⚠️ Maybe - Max steering angle |
| `pathplanning.py` | Calculates parking path | ❌ NO - Complex math |
| `environment.py` | Draws everything | ⚠️ Maybe - Colors only |

---

## 🚀 Common Changes

### Make parking easier:
```python
PARKING_LENGTH = 15  # Bigger spot
```

### Make parking harder:
```python
PARKING_LENGTH = 10  # Smaller spot
```

### Start from different angle:
```python
PSI_START = -30  # Steeper angle
```

### Smaller car:
```python
CAR_LENGTH = 6
CAR_WIDTH = 3
```

### Faster GIF:
```python
# Line 195: Change duration=0.1 to duration=0.05
```

---

## ❓ Common Questions

**Q: Where do I change the starting position?**  
A: Lines 39-41 in `simulate.py`

**Q: How do I make the parking spot bigger?**  
A: Line 48 in `simulate.py` - increase `PARKING_LENGTH`

**Q: How do I change the car size?**  
A: Lines 52-53 in `simulate.py`

**Q: Why does the car sometimes do multiple maneuvers?**  
A: Tight parking spots require multiple back-and-forth moves. Make `PARKING_LENGTH` bigger to avoid this.

**Q: Where are the outputs saved?**  
A: `reports/` folder - `parking_maneuver.gif` and `reference_trajectory.png`

**Q: Can I change the car color?**  
A: Yes! Line 17 in `environment.py` - change RGB values

---

## 📖 For More Details

See `CUSTOMIZATION_GUIDE.md` for:
- Detailed explanations of every part
- All customization options
- Troubleshooting
- Understanding confusing parts

---

## 🎓 For Your Case Study

**What to show:**
1. The parameters section (lines 35-59) - "These control everything"
2. Run the simulation - "Watch it park automatically"
3. Show the GIF - "Here's the result"
4. Explain: "It plans the path using geometry, then simulates the physics"

**What NOT to worry about:**
- Complex math in pathplanning.py
- Collision detection algorithms
- Coordinate transformations

**Focus on:**
- The parameters you can change
- The automatic path planning
- The visual results

---

## 💡 Pro Tips

1. **Change one thing at a time** - Easier to see what each change does
2. **Run after each change** - Make sure it still works
3. **Check the GIF** - Visual feedback is best
4. **Read error messages** - They usually tell you what's wrong
5. **Start with parameters** - Don't touch other files unless needed

---

**Remember:** 95% of customizations are in `simulate.py` lines 35-59!

