# Robust Quadrotor Flight Control

Development and simulation of a nonlinear 6-DOF quadrotor model, with planned implementation of PID, LQR and MPC controllers and EKF-based state estimation.

## Objectives

- Model the nonlinear translational and rotational dynamics of a quadrotor.
- Develop and compare classical, optimal and predictive control strategies.
- Implement state estimation using simulated sensor measurements.
- Evaluate tracking performance and robustness under disturbances.
- Extend the implementation toward C++ and PX4/Gazebo SITL.

## Current Progress

### Completed
- Nonlinear 12-state quadrotor dynamics in Python.
- Body-to-inertial rotation matrix using ZYX Euler angles.
- Open-loop simulation using SciPy `solve_ivp`.
- Hover equilibrium validation.
- Free-fall simulation.
- Initial roll perturbation test (5°).
- Visualization of altitude and horizontal displacement.

### Planned
- PID attitude and altitude control.
- LQR controller design and linearization.
- Model Predictive Control (MPC).
- Extended Kalman Filter (EKF).
- Robustness analysis.
- C++ implementation and PX4/Gazebo SITL validation.

## Technologies

Python, NumPy, SciPy, Matplotlib, Python Control.

## Run the Open-Loop Simulation

```bash
pip install -r requirements.txt
python simulation/open_loop.py
```

## Project Status

In active development.