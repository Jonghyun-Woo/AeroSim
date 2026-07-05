"""Top-level Aerosonde 6-DOF simulation with a successive-loop-closure autopilot."""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt

from ctrls.autopilot import Autopilot
from models.aerosonde import Aerosonde
from models.base_vehicle import RigidBody6DOF
from sim.config import load_config
from sim.logger import Logger


def commands_at(t):
    course = np.deg2rad(45.0) if t >= 10.0 else 0.0
    altitude = 120.0 if t >= 30.0 else 100.0
    airspeed = 28.0 if t >= 50.0 else 25.0
    return {"course": course, "altitude": altitude, "airspeed": airspeed}


def main():
    cfg = load_config("cfg/aerosonde.yaml")
    ap_cfg = load_config("cfg/autopilot.yaml")

    dt = ap_cfg["dt"]
    sim_time = 80.0
    num_steps = int(sim_time / dt) + 1

    placeholder_state = np.zeros(13)
    placeholder_state[6] = 1.0  # identity quaternion, avoids norm-zero on init
    trim_model = Aerosonde(cfg, placeholder_state, dt)
    trim_state, trim_controls = trim_model.compute_trim(Va=25.0, gamma=0.0)
    print(f"Trim residual cost: {trim_model.trim_cost:.3e}")
    print(f"Trim controls [de, da, dr, dt]: {trim_controls}")

    trim_state[2] = -100.0  # start at 100 m altitude

    plane = Aerosonde(cfg, trim_state, dt)
    autopilot = Autopilot(ap_cfg, trim_controls)

    wind_ned = np.zeros(3)

    state_logger = Logger(13, num_steps)
    control_logger = Logger(4, num_steps)
    command_logger = Logger(3, num_steps)
    airdata_logger = Logger(3, num_steps)

    time = 0.0
    for step in range(num_steps):
        wind_body = RigidBody6DOF.quat_to_rotm(plane.quaternion).T @ wind_ned
        air_data = Aerosonde.air_data(plane.state, wind_body)
        commands = commands_at(time)

        controls = autopilot.update(plane.state, commands, air_data)

        state_logger.update(plane.state, step, time)
        control_logger.update(controls, step, time)
        command_logger.update(
            [commands["course"], commands["altitude"], commands["airspeed"]], step, time
        )
        airdata_logger.update(air_data, step, time)

        plane.step(controls, wind_body)
        time += dt

    for logger in (state_logger, control_logger, command_logger, airdata_logger):
        logger.truncate()

    quats = state_logger.log[6:10, :].T
    vels_body = state_logger.log[3:6, :].T
    euler = np.array([RigidBody6DOF.quat_to_euler(q) for q in quats]).T
    vel_i = np.array([RigidBody6DOF.quat_to_rotm(q) @ v for q, v in zip(quats, vels_body)])
    chi = np.arctan2(vel_i[:, 1], vel_i[:, 0])
    altitude = -state_logger.log[2, :]

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    ax.plot(state_logger.log[0, :], state_logger.log[1, :], altitude)
    ax.set_xlabel("North (m)")
    ax.set_ylabel("East (m)")
    ax.set_zlabel("Altitude (m)")
    ax.set_title("3D Trajectory")

    fig, axs = plt.subplots(3, 1, sharex=True)
    axs[0].plot(state_logger.time, state_logger.log[0, :])
    axs[0].set_ylabel("pn (m)")
    axs[1].plot(state_logger.time, state_logger.log[1, :])
    axs[1].set_ylabel("pe (m)")
    axs[2].plot(state_logger.time, altitude, label="state")
    axs[2].plot(command_logger.time, command_logger.log[1, :], "--", label="command")
    axs[2].set_ylabel("altitude (m)")
    axs[2].set_xlabel("time (s)")
    axs[2].legend()
    fig.suptitle("Position")

    fig, axs = plt.subplots(3, 1, sharex=True)
    axs[0].plot(airdata_logger.time, airdata_logger.log[0, :], label="state")
    axs[0].plot(command_logger.time, command_logger.log[2, :], "--", label="command")
    axs[0].set_ylabel("Va (m/s)")
    axs[0].legend()
    axs[1].plot(airdata_logger.time, np.rad2deg(airdata_logger.log[1, :]))
    axs[1].set_ylabel("alpha (deg)")
    axs[2].plot(airdata_logger.time, np.rad2deg(airdata_logger.log[2, :]))
    axs[2].set_ylabel("beta (deg)")
    axs[2].set_xlabel("time (s)")
    fig.suptitle("Air data")

    fig, axs = plt.subplots(3, 1, sharex=True)
    axs[0].plot(state_logger.time, np.rad2deg(euler[0, :]))
    axs[0].set_ylabel("roll (deg)")
    axs[1].plot(state_logger.time, np.rad2deg(euler[1, :]))
    axs[1].set_ylabel("pitch (deg)")
    axs[2].plot(state_logger.time, np.rad2deg(chi), label="state")
    axs[2].plot(command_logger.time, np.rad2deg(command_logger.log[0, :]), "--", label="command")
    axs[2].set_ylabel("course (deg)")
    axs[2].set_xlabel("time (s)")
    axs[2].legend()
    fig.suptitle("Attitude / Course")

    fig, axs = plt.subplots(4, 1, sharex=True)
    axs[0].plot(control_logger.time, np.rad2deg(control_logger.log[0, :]))
    axs[0].set_ylabel("elevator (deg)")
    axs[1].plot(control_logger.time, np.rad2deg(control_logger.log[1, :]))
    axs[1].set_ylabel("aileron (deg)")
    axs[2].plot(control_logger.time, np.rad2deg(control_logger.log[2, :]))
    axs[2].set_ylabel("rudder (deg)")
    axs[3].plot(control_logger.time, control_logger.log[3, :])
    axs[3].set_ylabel("throttle")
    axs[3].set_xlabel("time (s)")
    fig.suptitle("Control surfaces")

    plt.show()


if __name__ == "__main__":
    main()
