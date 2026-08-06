"""Generic 6-DOF rigid-body kinematics/dynamics shared by all vehicle models."""
from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

STATE_SIZE = 13


class RigidBody6DOF(ABC):
    """Generic rigid-body 6-DOF vehicle.

    State vector (13,): [pn, pe, pd, u, v, w, e0, ex, ey, ez, p, q, r].
    Position (pn,pe,pd) and orientation quaternion (e0,ex,ey,ez) are in the
    inertial (NED) frame; velocity (u,v,w) and angular rate (p,q,r) are in
    the body frame. Subclasses implement forces_moments() to inject
    vehicle-specific aerodynamics/propulsion; gravity and the rigid-body
    kinematics/dynamics equations are handled here so they aren't
    re-derived by every vehicle type.
    """

    def __init__(self, mass, inertia, initial_state, dt, gravity=9.81, rk4_substeps=10):
        self.mass = float(mass)
        self.J = np.asarray(inertia, dtype=float).reshape(3, 3)
        self.Jinv = np.linalg.inv(self.J)
        self.gravity = float(gravity)
        self.dt = float(dt)
        self.rk4_substeps = int(rk4_substeps)

        self.state = np.asarray(initial_state, dtype=float).reshape(STATE_SIZE).copy()
        self.state[6:10] /= np.linalg.norm(self.state[6:10])

    # ---- convenience views into the current state ----
    @property
    def position(self):
        return self.state[0:3]

    @property
    def velocity_body(self):
        return self.state[3:6]

    @property
    def quaternion(self):
        return self.state[6:10]

    @property
    def omega(self):
        return self.state[10:13]

    @property
    def euler(self):
        return self.quat_to_euler(self.quaternion)

    def step(self, controls, wind_body=None):
        """Advance the state by self.dt using RK4.

        wind_body is held constant across all RK4 sub-steps within this call
        (mirrors the old MATLAB MultiCopter.m convention). Revisit this if a
        gust model sampled at sub-step resolution is added later.
        """
        controls = np.asarray(controls, dtype=float)
        wind_body = np.zeros(3) if wind_body is None else np.asarray(wind_body, dtype=float)

        state = self.state
        dt_rk = self.dt / self.rk4_substeps
        for _ in range(self.rk4_substeps):
            k1 = self._state_derivative(state, controls, wind_body)
            k2 = self._state_derivative(state + 0.5 * dt_rk * k1, controls, wind_body)
            k3 = self._state_derivative(state + 0.5 * dt_rk * k2, controls, wind_body)
            k4 = self._state_derivative(state + dt_rk * k3, controls, wind_body)
            state = state + (dt_rk / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
            state[6:10] /= np.linalg.norm(state[6:10])

        self.state = state
        return self.state

    def _state_derivative(self, state, controls, wind_body):
        vel = state[3:6]
        quat = state[6:10]
        omega = state[10:13]
        p, q, r = omega

        fm = self.forces_moments(state, controls, wind_body)
        force = fm[0:3]
        moment = fm[3:6]

        # Gravity is generic to any 6-DOF rigid body, so it's added here
        # rather than inside every subclass's forces_moments().
        f_gravity = self.quat_to_rotm(quat).T @ np.array([0.0, 0.0, self.mass * self.gravity])

        pos_dot = self.quat_to_rotm(quat) @ vel
        u, v, w = vel
        vel_dot = np.array([r * v - q * w, p * w - r * u, q * u - p * v]) + (force + f_gravity) / self.mass
        quat_dot = 0.5 * self._omega_matrix(omega) @ quat
        omega_dot = self.Jinv @ (moment - np.cross(omega, self.J @ omega))

        return np.concatenate([pos_dot, vel_dot, quat_dot, omega_dot])

    @abstractmethod
    def forces_moments(self, state, controls, wind_body):
        """Return [fx, fy, fz, l, m, n] in body frame (N, N*m), excluding gravity."""
        raise NotImplementedError

    # ---- quaternion / Euler helpers (scalar-first quaternion [e0,ex,ey,ez]) ----
    @staticmethod
    def _omega_matrix(omega):
        p, q, r = omega
        return np.array([
            [0.0, -p, -q, -r],
            [p, 0.0, r, -q],
            [q, -r, 0.0, p],
            [r, q, -p, 0.0],
        ])

    @staticmethod
    def quat_to_rotm(quat):
        """Rotation matrix R_b2i: rotates a body-frame vector into the inertial frame."""
        e0, ex, ey, ez = quat
        return np.array([
            [e0**2 + ex**2 - ey**2 - ez**2, 2 * (ex * ey - e0 * ez), 2 * (ex * ez + e0 * ey)],
            [2 * (ex * ey + e0 * ez), e0**2 - ex**2 + ey**2 - ez**2, 2 * (ey * ez - e0 * ex)],
            [2 * (ex * ez - e0 * ey), 2 * (ey * ez + e0 * ex), e0**2 - ex**2 - ey**2 + ez**2],
        ])

    @staticmethod
    def quat_to_euler(quat):
        """Returns [phi, theta, psi] (roll, pitch, yaw), aerospace 3-2-1 sequence."""
        e0, ex, ey, ez = quat
        phi = np.arctan2(2 * (e0 * ex + ey * ez), 1 - 2 * (ex**2 + ey**2))
        theta = np.arcsin(np.clip(2 * (e0 * ey - ez * ex), -1.0, 1.0))
        psi = np.arctan2(2 * (e0 * ez + ex * ey), 1 - 2 * (ey**2 + ez**2))
        return np.array([phi, theta, psi])

    @staticmethod
    def euler_to_quat(euler):
        """euler = [phi, theta, psi] (roll, pitch, yaw) -> [e0, ex, ey, ez]."""
        phi, theta, psi = euler
        cr, sr = np.cos(phi / 2), np.sin(phi / 2)
        cp, sp = np.cos(theta / 2), np.sin(theta / 2)
        cy, sy = np.cos(psi / 2), np.sin(psi / 2)
        return np.array([
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ])
