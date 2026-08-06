"""Aerosonde fixed-wing UAV: aerodynamic/propulsion forces & moments, plus trim.

Equations follow Beard & McLain, "Small Unmanned Aircraft: Theory and
Practice" (UAVBook), Ch.4 (forces and moments) and Ch.5 (trim).
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import minimize

from models.base_vehicle import RigidBody6DOF

_MIN_AIRSPEED = 1e-6


class Aerosonde(RigidBody6DOF):
    def __init__(self, params, initial_state, dt, rk4_substeps=10):
        Jx, Jy, Jz, Jxz = params["Jx"], params["Jy"], params["Jz"], params["Jxz"]
        inertia = np.array([
            [Jx, 0.0, -Jxz],
            [0.0, Jy, 0.0],
            [-Jxz, 0.0, Jz],
        ])
        super().__init__(
            mass=params["mass"],
            inertia=inertia,
            initial_state=initial_state,
            dt=dt,
            gravity=params.get("gravity", 9.81),
            rk4_substeps=rk4_substeps,
        )
        self.p = params
        self.trim_cost = None

    @staticmethod
    def air_data(state, wind_body):
        """Relative airspeed/alpha/beta from body-frame velocity and wind."""
        vr = state[3:6] - np.asarray(wind_body, dtype=float)
        ur, vr_lat, wr = vr
        Va = float(np.linalg.norm(vr))
        alpha = float(np.arctan2(wr, ur))
        beta = float(np.arcsin(np.clip(vr_lat / Va, -1.0, 1.0))) if Va > _MIN_AIRSPEED else 0.0
        return Va, alpha, beta

    @staticmethod
    def _aero_coeffs_longitudinal(alpha, CL0, CL_alpha, CD0, CM0, CM_alpha, e_osw, AR):
        """Baseline (rate/control-independent) lift/drag/pitch coefficients at alpha."""
        CL = CL0 + CL_alpha * alpha
        CD = CD0 + CL**2 / (np.pi * e_osw * AR)
        Cm = CM0 + CM_alpha * alpha
        return CL, CD, Cm

    def forces_moments(self, state, controls, wind_body):
        delta_e, delta_a, delta_r, delta_t = controls
        p, q, r = state[10:13]

        Va, alpha, beta = self.air_data(state, wind_body)
        Va_safe = max(Va, _MIN_AIRSPEED)

        geom = self.p["geometry"]
        lon = self.p["aero_longitudinal"]
        lat = self.p["aero_lateral"]
        prop = self.p["propulsion"]
        rho = self.p["atmosphere"]["rho"]

        S, b, c, AR, e_osw = geom["S"], geom["b"], geom["c"], geom["AR"], geom["e_osw"]
        qbar = 0.5 * rho * Va**2

        CL, CD, Cm0_alpha = self._aero_coeffs_longitudinal(
            alpha, lon["CL0"], lon["CL_alpha"], lon["CD0"], lon["CM0"], lon["CM_alpha"], e_osw, AR
        )
        CL_dyn = CL + lon["CL_q"] * (c / (2 * Va_safe)) * q + lon["CL_delta_e"] * delta_e
        CD_dyn = CD + lon["CD_q"] * (c / (2 * Va_safe)) * q + lon["CD_delta_e"] * delta_e
        Cm_dyn = Cm0_alpha + lon["CM_q"] * (c / (2 * Va_safe)) * q + lon["CM_delta_e"] * delta_e

        F_lift = qbar * S * CL_dyn
        F_drag = qbar * S * CD_dyn
        fx_long = -F_drag * np.cos(alpha) + F_lift * np.sin(alpha)
        fz_long = -F_drag * np.sin(alpha) - F_lift * np.cos(alpha)
        m_pitch = qbar * S * c * Cm_dyn

        CY_dyn = (
            lat["CY0"] + lat["CY_beta"] * beta
            + lat["CY_p"] * (b / (2 * Va_safe)) * p + lat["CY_r"] * (b / (2 * Va_safe)) * r
            + lat["CY_delta_a"] * delta_a + lat["CY_delta_r"] * delta_r
        )
        Cl_dyn = (
            lat["Cl0"] + lat["Cl_beta"] * beta
            + lat["Cl_p"] * (b / (2 * Va_safe)) * p + lat["Cl_r"] * (b / (2 * Va_safe)) * r
            + lat["Cl_delta_a"] * delta_a + lat["Cl_delta_r"] * delta_r
        )
        Cn_dyn = (
            lat["Cn0"] + lat["Cn_beta"] * beta
            + lat["Cn_p"] * (b / (2 * Va_safe)) * p + lat["Cn_r"] * (b / (2 * Va_safe)) * r
            + lat["Cn_delta_a"] * delta_a + lat["Cn_delta_r"] * delta_r
        )
        fy = qbar * S * CY_dyn
        l_roll = qbar * S * b * Cl_dyn
        n_yaw = qbar * S * b * Cn_dyn

        thrust = 0.5 * rho * prop["S_prop"] * prop["C_prop"] * ((prop["k_motor"] * delta_t) ** 2 - Va**2)
        torque = -prop["k_Tp"] * (prop["k_Omega"] * delta_t) ** 2

        fx = fx_long + thrust
        fz = fz_long
        l_total = l_roll + torque
        m_total = m_pitch
        n_total = n_yaw

        return np.array([fx, fy, fz, l_total, m_total, n_total])

    def compute_trim(self, Va, gamma, R=float("inf"), max_iter=500):
        """Solve for steady wings-level or coordinated-turn-climb trim.

        Free variables: alpha, beta, phi, delta_e, delta_a, delta_r, delta_t.
        theta, p, q, r are derived from (alpha, phi, Va, gamma, R) via the
        standard coordinated-turn relations (UAVBook Ch.5); the cost function
        drives pd_dot to the requested climb rate and u_dot, v_dot, w_dot,
        p_dot, q_dot, r_dot to zero. pn_dot/pe_dot/heading rate are left free
        (they're expected to be nonzero for a turn).
        Returns (trim_state (13,), trim_controls (4,)).
        """
        chi_dot = 0.0 if np.isinf(R) else Va * np.cos(gamma) / R
        target_pd_dot = -Va * np.sin(gamma)

        def build(x):
            alpha, beta, phi, de, da, dr, dt = x
            theta = alpha + gamma
            p = -chi_dot * np.sin(theta)
            q = chi_dot * np.sin(phi) * np.cos(theta)
            r = chi_dot * np.cos(phi) * np.cos(theta)
            u = Va * np.cos(alpha) * np.cos(beta)
            v = Va * np.sin(beta)
            w = Va * np.sin(alpha) * np.cos(beta)
            quat = self.euler_to_quat(np.array([phi, theta, 0.0]))
            state = np.concatenate([[0.0, 0.0, 0.0], [u, v, w], quat, [p, q, r]])
            controls = np.array([de, da, dr, dt])
            return state, controls

        def cost(x):
            state, controls = build(x)
            xdot = self._state_derivative(state, controls, np.zeros(3))
            residual = np.array([
                xdot[2] - target_pd_dot,
                xdot[3], xdot[4], xdot[5],
                xdot[10], xdot[11], xdot[12],
            ])
            return float(np.sum(residual**2))

        x0 = np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5])
        bounds = [
            (-0.5, 0.5), (-0.3, 0.3), (-1.0, 1.0),
            (-0.7854, 0.7854), (-0.5236, 0.5236), (-0.5236, 0.5236),
            (0.0, 1.0),
        ]
        result = minimize(cost, x0, method="SLSQP", bounds=bounds,
                           options={"maxiter": max_iter, "ftol": 1e-12})
        self.trim_cost = result.fun
        trim_state, trim_controls = build(result.x)
        return trim_state, trim_controls
