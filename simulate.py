"""Non-interactive simulation with automatic plot and GIF generation."""
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

from environment import Environment, Parking1
from control import Car_Dynamics, MPC_Controller
from pathplanning import PathPlanning
from utils import DataLogger

if __name__ == '__main__':
    x_start = 75
    y_start = 40
    psi_start = -20
    x_end = 90
    y_end = 80
    parking_length = 12
    parking_margin = 1
    last_backward_length = 0
    use_control = False

    start = np.array([x_start, y_start])
    end = np.array([x_end, y_end])

    car_length = 8
    car_width = 4
    wheel_length = 1.5
    wheel_width = 0.7
    wheel_positions = np.array(
        [[2.0, 1.0], [2.0, -1.0], [-2.0, 1.0], [-2.0, -1.0]])

    my_car = Car_Dynamics(
        start[0],
        start[1],
        0,
        np.deg2rad(psi_start),
        dt=0.2,
        car_length=car_length,
        car_width=car_width,
        wheel_length=wheel_length,
        wheel_width=wheel_width,
        wheel_positions=wheel_positions,
    )

    parking1 = Parking1(
        my_car,
        parking_length,
        parking_margin,
        last_backward_length)
    end, obs = parking1.generate_obstacles()

    env = Environment(obs, my_car, parking_margin)
    controller = MPC_Controller()

    print("="*70)
    print("Geometric Parallel Parking Simulation")
    print("="*70)
    print(f"Start: ({x_start}, {y_start}), Heading: {psi_start}°")
    print(f"End: ({end[0]:.1f}, {end[1]:.1f})")
    print(f"Parking length: {parking_length}m, Margin: {parking_margin}m")
    print(f"Car: {car_length}m x {car_width}m")
    print()

    path_planner = PathPlanning(obs, my_car, parking1)
    print("Planning path...")
    path, steer = path_planner.plan_path(
        start[0] - my_car.a / 2 * np.cos(np.deg2rad(psi_start)),
        start[1] - my_car.a / 2 * np.sin(np.deg2rad(psi_start)),
        np.deg2rad(psi_start),
        end[0], end[1],
        last_backward_length)
    
    print(f"Path planned: {len(path)} points")
    print(f"Geometric constraints:")
    print(f"  R_Elmin: {path_planner.R_Elmin:.2f}m")
    print(f"  R_Bl_min: {path_planner.R_Bl_min:.2f}m")
    print(f"  L_min: {path_planner.L_min:.2f}m")
    print()

    env.draw_footprint(path)
    env.draw_path(path)

    logger = DataLogger()
    frames = []
    
    print("Simulating path following...")
    x, y, psi = my_car.x, my_car.y, my_car.psi
    delta = 0
    acc = 0
    v = 0
    collision_count = 0
    prev_v = 0
    
    initial_res = env.render(x, y, psi, 0)
    frames.append((initial_res * 255).astype(np.uint8))
    
    for i in range(len(path)):
        if use_control:
            vehicle_center_path = path_planner.get_vehicle_center_path(path)
            if i + 5 <= len(vehicle_center_path):
                acc, delta = controller.optimize(
                    my_car, vehicle_center_path[i:i + 5])
                my_car.update_state(my_car.move(acc, delta))
                x, y, psi = my_car.x, my_car.y, my_car.psi
                v = my_car.v
        else:
            delta = steer[i] if i < len(steer) else 0
            point = path[i]
            x = point[0]
            y = point[1]
            
            if i < len(path) - 1:
                v_vec = path[i + 1] - point
                if not np.any(v_vec == 0):
                    psi = np.arctan(v_vec[1] / v_vec[0])
                    v = np.linalg.norm(v_vec) / my_car.dt
                else:
                    v = 0
            else:
                v = 0
            
            if i > 0:
                acc = (v - prev_v) / my_car.dt
            else:
                acc = 0
            
            x += my_car.a / 2 * np.cos(psi)
            y += my_car.a / 2 * np.sin(psi)
            
            my_car.x = x
            my_car.y = y
            my_car.psi = psi
            my_car.v = v
            prev_v = v
        
        logger.log([x, y], my_car, acc, delta)
        res = env.render(x, y, psi, delta)
        frame = (res * 255).astype(np.uint8)
        frames.append(frame)
        
        is_collision = env.check_collision(x, y, psi)
        if is_collision:
            collision_count += 1
            print(f"  Collision detected at step {i}")

    os.makedirs('reports', exist_ok=True)
    final_res = env.render(x, y, psi, 0)
    cv2.imwrite('reports/reference_trajectory.png', (final_res * 255).astype(np.uint8))
    
    final_frame = (final_res * 255).astype(np.uint8)
    frames.append(final_frame)
    
    print("Creating animated GIF...")
    if HAS_IMAGEIO:
        gif_path = 'reports/parking_maneuver.gif'
        frame_skip = max(1, len(frames) // 200)
        gif_frames = frames[::frame_skip]
        gif_frames.extend([final_frame] * 5)
        try:
            imageio.mimwrite(gif_path, gif_frames, duration=0.1, loop=0)
        except AttributeError:
            imageio.mimsave(gif_path, gif_frames, duration=0.1, loop=0)
        print(f"  GIF saved to: {gif_path}")
    elif HAS_PIL:
        gif_path = 'reports/parking_maneuver.gif'
        frame_skip = max(1, len(frames) // 200)
        gif_frames = [Image.fromarray(frame) for frame in frames[::frame_skip]]
        gif_frames.extend([Image.fromarray(final_frame)] * 5)
        gif_frames[0].save(
            gif_path,
            save_all=True,
            append_images=gif_frames[1:],
            duration=100,
            loop=0
        )
        print(f"  GIF saved to: {gif_path}")
    
    print("Generating time series plots...")
    logger.save_data()
    
    print()
    print("="*70)
    print("SIMULATION RESULTS")
    print("="*70)
    print(f"Final position: ({x:.2f}, {y:.2f})")
    print(f"Final heading: {np.rad2deg(psi):.1f}°")
    print(f"Target position: ({end[0]:.2f}, {end[1]:.2f})")
    print(f"Position error: {np.linalg.norm([x - end[0], y - end[1]]):.2f}m")
    print(f"Collisions: {collision_count}")
    print(f"Path visualization saved to: reports/reference_trajectory.png")
    if HAS_IMAGEIO or HAS_PIL:
        print(f"Animated GIF saved to: reports/parking_maneuver.gif")
    print(f"Time series plots saved to: log results/")
    print("="*70)

