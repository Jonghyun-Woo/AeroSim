"""Generic PID / PD-with-rate-feedback control block."""
from __future__ import annotations

import numpy as np


class PID:
    """Single building block for both:
      - classic PID with numerically-differentiated error (rate=None), and
      - PD with direct rate feedback (rate=<measured rate>), as used by the
        autopilot's inner attitude loops (roll->aileron, pitch->elevator).

    u = kp*error + ki*integral(error) + D, where
      D = -kd*rate              if rate is given
      D = kd*d(error)/dt        otherwise (zero on the first call)
    """

    def __init__(self, kp, ki=0.0, kd=0.0, dt=0.01, u_min=-np.inf, u_max=np.inf):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.u_min = u_min
        self.u_max = u_max
        self.reset()

    def reset(self):
        self._err_prev = np.nan
        self._err_int = 0.0

    def update(self, reference, measurement, rate=None):
        error = reference - measurement

        P = self.kp * error
        I = self.ki * self._err_int
        if rate is not None:
            D = -self.kd * rate
        else:
            D = 0.0 if np.isnan(self._err_prev) else self.kd * (error - self._err_prev) / self.dt

        u_unsat = P + I + D
        u = np.clip(u_unsat, self.u_min, self.u_max)

        # Anti-windup: only integrate while not saturated.
        if u == u_unsat:
            self._err_int += error * self.dt

        self._err_prev = error
        return u
