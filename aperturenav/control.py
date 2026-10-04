"""Vector PID with anti-windup, derivative filtering and motion limits."""
import numpy as np

class PID:
    def __init__(self, config):
        self.c = config
        self.reset()

    def reset(self):
        self.integral = np.zeros(3)
        self.previous_error = None
        self.derivative = np.zeros(3)
        self.output = np.zeros(3)

    def step(self, error, dt, speed_limit=None):
        if dt <= 0:
            raise ValueError('dt must be positive')
        e = np.asarray(error, dtype=float)
        derivative = np.zeros(3) if self.previous_error is None else (e - self.previous_error) / dt
        alpha = dt / (0.08 + dt)
        self.derivative += alpha * (derivative - self.derivative)
        proposed = np.clip(self.integral + e*dt, -self.c['integral_limit'], self.c['integral_limit'])
        raw = self.c['kp']*e + self.c['ki']*proposed + self.c['kd']*self.derivative
        limit = self.c['max_speed'] if speed_limit is None else speed_limit
        norm = np.linalg.norm(raw)
        if norm <= limit:
            self.integral = proposed
        target = raw * min(1., limit / max(norm, 1e-12))
        delta = target - self.output
        delta *= min(1., self.c['max_acceleration']*dt / max(np.linalg.norm(delta), 1e-12))
        self.output += delta
        # Clamp after deceleration when entering a more restrictive speed mode.
        self.output *= min(1., limit / max(np.linalg.norm(self.output), 1e-12))
        self.previous_error = e.copy()
        return self.output.copy()
