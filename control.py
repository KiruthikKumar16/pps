import numpy as np
from scipy.optimize import minimize
import copy


class Car_Dynamics:
    """
    Kinematic Bicycle Model for a Car-like Vehicle
    
    This class implements a bicycle model (single-track model) that captures the
    essential nonholonomic constraints of a car. The model assumes:
    1. Vehicle moves on a flat plane (no roll/pitch)
    2. No wheel slip (nonholonomic constraint)
    3. Front wheels can be steered, rear wheels are fixed
    
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
            Format: [[front_right], [front_left], [rear_right], [rear_left]]
        """
        self.dt = dt
        self.L = wheel_positions[0][0] - wheel_positions[2][0]
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

        self.d_front = car_length / 2 - wheel_positions[0][0]
        self.d_rear = car_length / 2 - (-wheel_positions[2][0])
        self.a = car_length - self.d_rear - self.d_front
        self.d_l = car_width / 2 - wheel_positions[0][1]
        self.d_r = car_width / 2 - (-wheel_positions[1][1])
        self.b = (car_width - self.d_l - self.d_r) / 2
        self.steer_max = np.deg2rad(40)

    def move(self, accelerate, delta):
        """
        Compute state derivatives using kinematic bicycle model.
        
        The kinematic equations are:
        - ẋ = v · cos(ψ)  : x-velocity component
        - ẏ = v · sin(ψ)  : y-velocity component  
        - v̇ = a            : acceleration
        - ψ̇ = (v · tan(δ)) / L  : yaw rate (from nonholonomic constraint)
        
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


class MPC_Controller:
    """Model Predictive Controller for trajectory tracking."""
    def __init__(self):
        self.horiz = None
        self.R = np.diag([0.01, 0.01])
        self.Rd = np.diag([0.01, 1.0])
        self.Q = np.diag([1.0, 1.0])
        self.Qf = self.Q

    def mpc_cost(self, u_k, my_car, points):
        mpc_car = copy.copy(my_car)
        u_k = u_k.reshape(self.horiz, 2).T
        z_k = np.zeros((2, self.horiz + 1))

        desired_state = points.T
        cost = 0.0

        for i in range(self.horiz):
            state_dot = mpc_car.move(u_k[0, i], u_k[1, i])
            mpc_car.update_state(state_dot)

            z_k[:, i] = [mpc_car.x, mpc_car.y]
            cost += np.sum(self.R @ (u_k[:, i]**2))
            cost += np.sum(self.Q @ ((desired_state[:, i] - z_k[:, i])**2))
            if i < (self.horiz - 1):
                cost += np.sum(self.Rd @ ((u_k[:, i + 1] - u_k[:, i])**2))
        return cost

    def optimize(self, my_car, points):
        self.horiz = points.shape[0]
        bnd = [(-5, 5), (np.deg2rad(-60), np.deg2rad(60))] * self.horiz
        result = minimize(
            self.mpc_cost, args=(
                my_car, points), x0=np.zeros(
                (2 * self.horiz)), method='SLSQP', bounds=bnd)
        return result.x[0], result.x[1]


class Linear_MPC_Controller:
    """Linearized Model Predictive Controller."""
    def __init__(self):
        self.horiz = None
        self.R = np.diag([0.01, 0.01])                 # input cost matrix
        self.Rd = np.diag([0.01, 1.0])                 # input difference cost matrix
        self.Q = np.diag([1.0, 1.0])                   # state cost matrix
        self.Qf = self.Q                               # state final matrix
        self.dt = 0.2
        self.L = 4

    def make_model(self, v, psi, delta):
        A = np.array([[1, 0, self.dt*np.cos(psi)         , -self.dt*v*np.sin(psi)],
                    [0, 1, self.dt*np.sin(psi)         , self.dt*v*np.cos(psi) ],
                    [0, 0, 1                           , 0                     ],
                    [0, 0, self.dt*np.tan(delta)/self.L, 1                     ]])
        B = np.array([[0      , 0                                  ],
                    [0      , 0                                  ],
                    [self.dt, 0                                  ],
                    [0      , self.dt*v/(self.L*np.cos(delta)**2)]])
        C = np.array([[self.dt*v* np.sin(psi)*psi                ],
                    [-self.dt*v*np.cos(psi)*psi                ],
                    [0                                         ],
                    [-self.dt*v*delta/(self.L*np.cos(delta)**2)]])

        return A, B, C

    def mpc_cost(self, u_k, my_car, points):

        u_k = u_k.reshape(self.horiz, 2).T
        z_k = np.zeros((2, self.horiz + 1))
        desired_state = points.T
        cost = 0.0
        old_state = np.array(
            [my_car.x, my_car.y, my_car.v, my_car.psi]).reshape(4, 1)

        for i in range(self.horiz):
            delta = u_k[1, i]
            A, B, C = self.make_model(my_car.v, my_car.psi, delta)
            new_state = A @ old_state + B @ u_k + C

            z_k[:, i] = [new_state[0, 0], new_state[1, 0]]
            cost += np.sum(self.R @ (u_k[:, i]**2))
            cost += np.sum(self.Q @ ((desired_state[:, i] - z_k[:, i])**2))
            if i < (self.horiz - 1):
                cost += np.sum(self.Rd @ ((u_k[:, i + 1] - u_k[:, i])**2))

            old_state = new_state
        return cost

    def optimize(self, my_car, points):
        self.horiz = points.shape[0]
        bnd = [(-5, 5), (np.deg2rad(-60), np.deg2rad(60))] * self.horiz
        result = minimize(
            self.mpc_cost, args=(
                my_car, points), x0=np.zeros(
                (2 * self.horiz)), method='SLSQP', bounds=bnd)
        return result.x[0], result.x[1]
