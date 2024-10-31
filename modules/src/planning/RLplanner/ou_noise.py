import numpy as np

class OrnsteinUhlenbeckNoise:
    def __init__(self, dim, mu=0.0, theta=0.01, sigma=0.01, dt=1e-1, x0=None):
        self.mu = mu
        self.theta = theta
        self.sigma = sigma
        self.dt = dt
        self.dim = dim
        self.x0 = x0
        self.reset()

    def reset(self):
        self.x_prev = self.x0 if self.x0 is not None else np.zeros(self.dim)

    def __call__(self):
        x = self.x_prev + self.theta * (self.mu - self.x_prev) * self.dt + self.sigma * np.sqrt(self.dt) * np.random.randn(*self.dim)
        self.x_prev = x
        return x