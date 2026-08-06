"""UAVBook Ch.6 successive-loop-closure autopilot for the Aerosonde."""
from __future__ import annotations

import numpy as np

from ctrls.pid import PID
from models.base_vehicle import RigidBody6DOF


class Autopilot:
    """Six-loop successive-loop-closure autopilot.

    roll (aileron) <- course (roll command) <- course command
    pitch (elevator) <- altitude (pitch command) <- altitude command
    sideslip (rudder) <- 0
    airspeed (throttle) <- airspeed command
    """

    _LOOP_NAMES = (
        "roll_from_aileron",
        "course_from_roll",
        "sideslip_from_rudder",
        "pitch_from_elevator",
        "altitude_from_pitch",
        "airspeed_from_throttle",
    )

    def __init__(self, gains, trim_controls, dt=None):
        dt = gains.get("dt", dt)
        for name in self._LOOP_NAMES:
            setattr(self, name, PID(dt=dt, **gains[name]))
        self.trim_controls = np.asarray(trim_controls, dtype=float)

    def update(self, state, commands, air_data):
        """commands: {"course": rad, "altitude": m, "airspeed": m/s}."""
        Va, alpha, beta = air_data
        phi, theta, psi = RigidBody6DOF.quat_to_euler(state[6:10])
        p, q, r = state[10:13]
        pd = state[2]

        vel_i = RigidBody6DOF.quat_to_rotm(state[6:10]) @ state[3:6]
        chi = np.arctan2(vel_i[1], vel_i[0])

        roll_c = self.course_from_roll.update(commands["course"], chi)
        delta_a = self.roll_from_aileron.update(roll_c, phi, rate=p)
        delta_r = self.sideslip_from_rudder.update(0.0, beta)

        pitch_c = self.altitude_from_pitch.update(commands["altitude"], -pd)
        delta_e = self.trim_controls[0] + self.pitch_from_elevator.update(pitch_c, theta, rate=q)
        delta_t = self.trim_controls[3] + self.airspeed_from_throttle.update(commands["airspeed"], Va)
        delta_t = float(np.clip(delta_t, 0.0, 1.0))

        return np.array([delta_e, delta_a, delta_r, delta_t])
