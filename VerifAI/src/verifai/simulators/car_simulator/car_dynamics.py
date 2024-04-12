import numpy as np

class dynamics:
    def __init__(self, nx, nu, f, dt = 0.1):
        self.nx = nx
        self.nu = nu
        self.f = f
        self.dt = dt
    def next_state(self, x, u):
        return x + self.f(x, u) * self.dt

class car_dynamics(dynamics):
    def __init__(self, wheelbase, dt=0.1):
        self.wheelbase = wheelbase

        def f(x, u):
            return np.array([x[2] * np.sin(float(np.pi/2 - x[3])),
                       x[2] * np.cos(float(np.pi/2 - x[3])),
                       u[1],
                       -x[2] * np.tan(float(u[0])) / self.wheelbase])

        super().__init__(nx=4, nu=2, f=f, dt=dt)



