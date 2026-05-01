from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PIDController:
    kp: float
    ki: float
    kd: float
    dt: float
    integrator_limit: float | None = None
    output_limit: float | None = None

    _i: float = 0.0
    _prev_error: float = 0.0
    _initialized: bool = False

    def reset(self) -> None:
        self._i = 0.0
        self._prev_error = 0.0
        self._initialized = False

    def update(self, error: float) -> float:
        if not self._initialized:
            self._prev_error = error
            self._initialized = True

        # Integrator
        self._i += error * self.dt
        if self.integrator_limit is not None:
            if self._i > self.integrator_limit:
                self._i = self.integrator_limit
            elif self._i < -self.integrator_limit:
                self._i = -self.integrator_limit

        # Derivative of error
        d = (error - self._prev_error) / self.dt
        self._prev_error = error

        u = self.kp * error + self.ki * self._i + self.kd * d

        if self.output_limit is not None:
            if u > self.output_limit:
                u = self.output_limit
            elif u < -self.output_limit:
                u = -self.output_limit

        return u


@dataclass
class LADRCController:
    """
    Linear ADRC for a (nominal) 2nd-order plant per axis:
      y_dot  = x2
      x2_dot = b0*u + d  (d is total disturbance)

    ESO states:
      z1 ~ y, z2 ~ y_dot, z3 ~ d

    Discrete-time ESO update (Euler):
      e  = y - z1
      z1 += dt*(z2 + beta1*e)
      z2 += dt*(z3 + b0*u_prev + beta2*e)
      z3 += dt*(beta3*e)

    Control law:
      u = (kp*(r - z1) - kd*z2 - z3)/b0
    """

    dt: float
    b0: float
    w0: float  # observer bandwidth (rad/s)
    kp: float
    kd: float
    u_limit: float | None = None

    z1: float = 0.0
    z2: float = 0.0
    z3: float = 0.0
    _u_prev: float = 0.0
    _initialized: bool = False

    def reset(self) -> None:
        self.z1 = 0.0
        self.z2 = 0.0
        self.z3 = 0.0
        self._u_prev = 0.0
        self._initialized = False

    def _betas(self) -> tuple[float, float, float]:
        beta1 = 3.0 * self.w0
        beta2 = 3.0 * (self.w0**2)
        beta3 = self.w0**3
        return beta1, beta2, beta3

    def update(self, y: float, r: float = 0.0) -> float:
        if not self._initialized:
            self.z1 = y
            self.z2 = 0.0
            self.z3 = 0.0
            self._u_prev = 0.0
            self._initialized = True

        beta1, beta2, beta3 = self._betas()

        e = y - self.z1
        self.z1 += self.dt * (self.z2 + beta1 * e)
        self.z2 += self.dt * (self.z3 + self.b0 * self._u_prev + beta2 * e)
        self.z3 += self.dt * (beta3 * e)

        u = (self.kp * (r - self.z1) - self.kd * self.z2 - self.z3) / self.b0

        if self.u_limit is not None:
            if u > self.u_limit:
                u = self.u_limit
            elif u < -self.u_limit:
                u = -self.u_limit

        self._u_prev = u
        return u

