"""
Simplified Parallel Parking Simulation
=======================================
This is a simplified version that demonstrates the core concepts:
- Geometric path planning using circular arcs
- Kinematic bicycle model for vehicle motion
- Collision detection
- Visualization

Run: python simulate.py
Output: 
  - reports/parking_maneuver.gif (animated parking)
  - reports/reference_trajectory.png (final trajectory)
"""
import cv2
import numpy as np
import os
try:
    import imageio
    HAS_IMAGEIO = True
except ImportError:
    try:
        from PIL import Image
        HAS_PIL = True
        HAS_IMAGEIO = False
    except ImportError:
        HAS_IMAGEIO = False
        HAS_PIL = False
        print("Warning: No GIF library found. Install imageio or Pillow for GIF generation.")

from environment import Environment, Parking1
from control import Car_Dynamics
from pathplanning import PathPlanning

# ============================================================================
# SIMULATION PARAMETERS (Hardcoded for simplicity)
# ============================================================================
# Starting position and orientation
X_START = 75
Y_START = 40
PSI_START = -20  # degrees

# Target parking position (will be adjusted by parking scenario)
X_END = 90
Y_END = 80

# Parking spot configuration
PARKING_LENGTH = 12  # meters
PARKING_MARGIN = 1   # meters

# Vehicle dimensions
CAR_LENGTH = 8   # meters
CAR_WIDTH = 4    # meters
WHEEL_LENGTH = 1.5
WHEEL_WIDTH = 0.7
WHEEL_POSITIONS = np.array([[2.0, 1.0], [2.0, -1.0], [-2.0, 1.0], [-2.0, -1.0]])

# Simulation time step
DT = 0.2  # seconds

# ============================================================================
# MAIN SIMULATION
# ============================================================================
if __name__ == '__main__':
    print("="*70)
    print("SIMPLIFIED PARALLEL PARKING SIMULATION")
    print("="*70)
    print(f"Start Position: ({X_START}, {Y_START})")
    print(f"Start Heading: {PSI_START}°")
    print(f"Parking Length: {PARKING_LENGTH}m")
    print(f"Car Size: {CAR_LENGTH}m x {CAR_WIDTH}m")
    print()
    
    # Initialize vehicle
    start = np.array([X_START, Y_START])
    my_car = Car_Dynamics(
        start[0], start[1], 0, np.deg2rad(PSI_START),
        dt=DT,
        car_length=CAR_LENGTH,
        car_width=CAR_WIDTH,
        wheel_length=WHEEL_LENGTH,
        wheel_width=WHEEL_WIDTH,
        wheel_positions=WHEEL_POSITIONS,
    )
    
    # Create parking scenario with obstacles
    parking = Parking1(my_car, PARKING_LENGTH, PARKING_MARGIN, 0)
    end, obstacles = parking.generate_obstacles()
    
    print(f"Target Position: ({end[0]:.1f}, {end[1]:.1f})")
    print()
    
    # Initialize environment
    env = Environment(obstacles, my_car, PARKING_MARGIN)
    
    # Plan the path
    print("Planning geometric path...")
    path_planner = PathPlanning(obstacles, my_car, parking)
    
    # Calculate starting point (rear axle position)
    start_rear_x = start[0] - my_car.a / 2 * np.cos(np.deg2rad(PSI_START))
    start_rear_y = start[1] - my_car.a / 2 * np.sin(np.deg2rad(PSI_START))
    
    # Generate path (uses one-trial for spacious spots, several-trial for tight spots)
    path, steer_angles = path_planner.plan_path(
        start_rear_x, start_rear_y, np.deg2rad(PSI_START),
        end[0], end[1], 0
    )
    
    print(f"✓ Path planned: {len(path)} waypoints")
    print(f"  Minimum turning radius: {path_planner.R_Elmin:.2f}m")
    print()
    
    # Draw planned path on background
    env.draw_footprint(path)
    env.draw_path(path)
    
    # ========================================================================
    # SIMULATE PATH FOLLOWING
    # ========================================================================
    print("Simulating vehicle motion...")
    frames = []
    collision_count = 0
    
    # Initial frame
    x, y, psi = my_car.x, my_car.y, my_car.psi
    initial_frame = env.render(x, y, psi, 0)
    frames.append((initial_frame * 255).astype(np.uint8))
    
    # Follow the planned path
    for i in range(len(path)):
        # Get steering angle for this waypoint
        delta = steer_angles[i] if i < len(steer_angles) else 0
        
        # Get position from path
        point = path[i]
        x = point[0]
        y = point[1]
        
        # Calculate heading from path direction
        if i < len(path) - 1:
            direction = path[i + 1] - point
            if not np.any(direction == 0):
                psi = np.arctan2(direction[1], direction[0])
        
        # Convert rear axle position to vehicle center
        x_center = x + my_car.a / 2 * np.cos(psi)
        y_center = y + my_car.a / 2 * np.sin(psi)
        
        # Update car state (for visualization)
        my_car.x = x_center
        my_car.y = y_center
        my_car.psi = psi
        
        # Render frame
        frame = env.render(x_center, y_center, psi, delta)
        frames.append((frame * 255).astype(np.uint8))
        
        # Check for collisions
        if env.check_collision(x_center, y_center, psi):
            collision_count += 1
            print(f"  ⚠ Collision detected at step {i}")
    
    print(f"✓ Simulation complete: {len(frames)} frames")
    print()
    
    # ========================================================================
    # SAVE OUTPUTS
    # ========================================================================
    os.makedirs('reports', exist_ok=True)
    
    # Save final trajectory image
    final_frame = env.render(x, y, psi, 0)
    cv2.imwrite('reports/reference_trajectory.png', 
                (final_frame * 255).astype(np.uint8))
    print("✓ Saved: reports/reference_trajectory.png")
    
    # Create animated GIF
    if HAS_IMAGEIO or HAS_PIL:
        print("Creating animated GIF...")
        gif_path = 'reports/parking_maneuver.gif'
        
        # Add final frame multiple times for pause
        final_frame_array = (final_frame * 255).astype(np.uint8)
        frames.append(final_frame_array)
        frames.append(final_frame_array)
        frames.append(final_frame_array)
        
        # Reduce frames for smaller file size
        frame_skip = max(1, len(frames) // 150)
        gif_frames = frames[::frame_skip]
        
        if HAS_IMAGEIO:
            try:
                imageio.mimwrite(gif_path, gif_frames, duration=0.1, loop=0)
            except AttributeError:
                imageio.mimsave(gif_path, gif_frames, duration=0.1, loop=0)
        elif HAS_PIL:
            pil_frames = [Image.fromarray(frame) for frame in gif_frames]
            pil_frames[0].save(
                gif_path,
                save_all=True,
                append_images=pil_frames[1:],
                duration=100,
                loop=0
            )
        print(f"✓ Saved: {gif_path}")
    else:
        print("⚠ Skipping GIF (install imageio or Pillow)")
    
    # ========================================================================
    # RESULTS SUMMARY
    # ========================================================================
    print()
    print("="*70)
    print("RESULTS")
    print("="*70)
    print(f"Final Position: ({x:.2f}, {y:.2f})")
    print(f"Target Position: ({end[0]:.2f}, {end[1]:.2f})")
    position_error = np.linalg.norm([x - end[0], y - end[1]])
    print(f"Position Error: {position_error:.2f}m")
    print(f"Collisions: {collision_count}")
    print()
    print("Output files:")
    print("  - reports/reference_trajectory.png")
    if HAS_IMAGEIO or HAS_PIL:
        print("  - reports/parking_maneuver.gif")
    print("="*70)

