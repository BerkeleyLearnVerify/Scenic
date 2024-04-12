# This file defines the car objects
from verifai.simulators.car_simulator.car_dynamics import car_dynamics
import numpy as np

class car:
    def __init__(self, x0, u_domain, color='yellow'):
        self.x0 = x0
        self.u_domain = u_domain
        self.color = color
        self.trajectory = [x0]
    def reset(self):
        self.trajectory = [self.x0]
        self.control_trajectory = []

class bicycle_model(car):
    def __init__(self, x0, u_domain, compute_control,
                 wheelbase=3.0, dt=0.1, color='yellow'):
        if u_domain is None:
            u_domain= {'omega':[-np.pi/4, np.pi/4], 'acc':[-4.0, 4.0]}
        elif 'omega' not in u_domain:
            u_domain['omega'] = [-np.pi/4, np.pi/4]
        elif 'acc' not in u_domain:
            u_domain['acc'] = [-4.0, 4.0]
        super().__init__(x0, u_domain, color)
        self.dynamics = car_dynamics(wheelbase=wheelbase, dt=dt)
        self.control_trajectory = []
        self.compute_control = compute_control

    def assert_control(self, u):
        u[0] = min(u[0], self.u_domain['omega'][1])
        u[0] = max(u[0], self.u_domain['omega'][0])

        u[1] = min(u[1], self.u_domain['acc'][1])
        u[1] = max(u[1], self.u_domain['acc'][0])

        return u

    def step(self):
        x = self.trajectory[-1]
        u = self.compute_control(self.trajectory, self.control_trajectory)
        u = self.assert_control(u)
        self.control_trajectory.append(u)
        self.trajectory.append(self.dynamics.next_state(x, u))

