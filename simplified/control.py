"""
Simplified Vehicle Dynamics Model
==================================
Kinematic Bicycle Model - simplified version without MPC controllers.
"""
import numpy as np


class Car_Dynamics:
    """
    Kinematic Bicycle Model for a Car-like Vehicle
    
    This implements a bicycle model (single-track model) that captures the
    essential nonholonomic constraints of a car.
    
    State Variables:
        x: x-position in global frame [m]
        y: y-position in global frame [m]
        v: linear velocity [m/s]
        psi: heading angle/yaw angle [rad]
    
    Input Variables:
        a: longitudinal acceleration [m/s²]
        delta: steering angle [rad]
    
    Kinematic Equations:
        ẋ = v · cos(ψ)
        ẏ = v · sin(ψ)
        v̇ = a
        ψ̇ = (v · tan(δ)) / L
    
    Where L is the wheelbase (distance between front and rear axles).
    """
    def __init__(self,
                 x_0,
                 y_0,
                 v_0,
                 psi_0,
                 dt,
                 car_length=8,
                 car_width=4,
                 wheel_length=1.5,
                 wheel_width=0.7,
                 wheel_positions=np.array(
                     [[2.0, 1.0], [2.0, -1.0], [-2.0, 1.0], [-2.0, -1.0]])
                 ):
        """
        Initialize the car dynamics model.
        
        Parameters:
        -----------
        x_0, y_0 : float
            Initial position in global frame [m]
        v_0 : float
            Initial velocity [m/s]
        psi_0 : float
            Initial heading angle [rad]
        dt : float
            Sampling time for numerical integration [s]
        car_length : float
            Total length of the vehicle [m]
        car_width : float
            Total width of the vehicle [m]
        wheel_length, wheel_width : float
            Dimensions of individual wheels [m]
        wheel_positions : np.array
            Positions of wheels relative to vehicle center [m]
        """
        self.dt = dt
        self.L = wheel_positions[0][0] - wheel_positions[2][0]  # wheelbase
        self.x = x_0
        self.y = y_0
        self.v = v_0
        self.psi = psi_0
        self.state = np.array([[self.x, self.y, self.v, self.psi]]).T

        self.car_length = car_length
        self.car_width = car_width
        self.wheel_length = wheel_length
        self.wheel_width = wheel_width
        self.wheel_positions = wheel_positions

        # Vehicle geometry calculations
        self.d_front = car_length / 2 - wheel_positions[0][0]
        self.d_rear = car_length / 2 - (-wheel_positions[2][0])
        self.a = car_length - self.d_rear - self.d_front  # distance between axles
        self.d_l = car_width / 2 - wheel_positions[0][1]
        self.d_r = car_width / 2 - (-wheel_positions[1][1])
        self.b = (car_width - self.d_l - self.d_r) / 2
        self.steer_max = np.deg2rad(40)  # maximum steering angle

    def move(self, accelerate, delta):
        """
        Compute state derivatives using kinematic bicycle model.
        
        Kinematic equations:
        - ẋ = v · cos(ψ)  : x-velocity component
        - ẏ = v · sin(ψ)  : y-velocity component  
        - v̇ = a            : acceleration
        - ψ̇ = (v · tan(δ)) / L  : yaw rate
        
        Parameters:
        -----------
        accelerate : float
            Longitudinal acceleration [m/s²]
        delta : float
            Steering angle [rad]
        
        Returns:
        --------
        state_dot : np.array
            State derivatives [ẋ, ẏ, v̇, ψ̇]ᵀ
        """
        x_dot = self.v * np.cos(self.psi)
        y_dot = self.v * np.sin(self.psi)
        v_dot = accelerate
        psi_dot = self.v * np.tan(delta) / self.L
        return np.array([[x_dot, y_dot, v_dot, psi_dot]]).T

    def update_state(self, state_dot):
        """
        Update vehicle state using Euler integration.
        
        State update: x(k+1) = x(k) + dt · ẋ(k)
        
        Parameters:
        -----------
        state_dot : np.array
            State derivatives [ẋ, ẏ, v̇, ψ̇]ᵀ from move() method
        """
        self.state = self.state + self.dt * state_dot
        self.x = self.state[0, 0]
        self.y = self.state[1, 0]
        self.v = self.state[2, 0]
        self.psi = self.state[3, 0]

