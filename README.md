# AeroSim

A Python simulation environment implementing a 6-DOF Aerosonde fixed-wing model
and a successive-loop-closure autopilot, based on Randy Beard & Tim McLain,
*Small Unmanned Aircraft: Theory and Practice* (UAVBook).

## Folder structure

```
AeroSim/
  environment.yml       # conda environment definition
  cfg/                  # per-model configuration (yaml)
    aerosonde.yaml       # Aerosonde airframe specs / aero & propulsion coefficients
    autopilot.yaml       # autopilot gains
  models/                # 6DOF models (shared base class + per-vehicle implementations)
    base_vehicle.py        # RigidBody6DOF: quaternion-based 6DOF kernel (shared by future multicopters etc.)
    aerosonde.py            # Aerosonde aero/propulsion model + trim solver
  ctrls/                 # controllers (add controllers other than PID here too)
    pid.py                  # combined PID / rate-feedback PD block
    autopilot.py            # Ch.6 successive-loop-closure autopilot
  sim/                   # simulation utilities: config loader, logger, etc.
  run_aerosonde_sim.py   # run script (trim -> simulation -> plot)
```

## Environment setup (conda)

This project manages its virtual environment with conda. You need
[Miniconda](https://docs.conda.io/en/latest/miniconda.html) or Anaconda installed.

```bash
# 1. Create the conda environment from the repo root (using environment.yml)
conda env create -f environment.yml

# 2. Activate the environment
conda activate aerosim
```

If `environment.yml` has been updated after the environment was created,
sync it with:

```bash
conda env update -f environment.yml --prune
```

To remove the environment:

```bash
conda deactivate
conda env remove -n aerosim
```

## Running

Run from the repo root (with the environment activated).

```bash
conda activate aerosim
python run_aerosonde_sim.py
```

The script proceeds in the following order:

1. Load the `cfg/aerosonde.yaml` and `cfg/autopilot.yaml` configurations
2. Compute trim for the specified airspeed (Va) and flight-path angle (gamma)
   with `Aerosonde.compute_trim()` (prints the trim residual and trim control
   surface / throttle values to the console)
3. Simulate starting from the trim state, stepping through course/altitude/airspeed
   commands in sequence
4. Display plots: 3D trajectory, position, air data (Va/alpha/beta),
   attitude/course, and control-surface time histories

## Notes

- The airframe specs and aerodynamic coefficients in `cfg/aerosonde.yaml` are
  based on the Aerosonde values published in UAVBook Appendix E. It is
  recommended to verify them against the book's appendix.
- The gains in `cfg/autopilot.yaml` are arbitrary starting values, not derived
  from the book's transfer-function-based gain design equations. Tune them by
  hand against the plots produced by `run_aerosonde_sim.py`.
- Features from the old MATLAB code — MultiCopter, the Dryden wind model, the
  disturbance observer (DOB), etc. — are not included in this Python port.
  `RigidBody6DOF` in `models/base_vehicle.py` is designed so other vehicle
  models can share it; add new subclasses under `models/` when needed.
