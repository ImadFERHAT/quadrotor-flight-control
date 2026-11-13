# Robust Quadrotor Flight Control

Development and simulation of a nonlinear 6-DOF quadrotor model, with planned implementation of PID, LQR and MPC controllers and EKF-based state estimation.

## Objectives

- Model the nonlinear translational and rotational dynamics of a quadrotor.
- Develop and compare classical, optimal and predictive control strategies.
- Implement state estimation using simulated sensor measurements.
- Evaluate tracking performance and robustness under disturbances.
- Extend the implementation toward C++ and PX4/Gazebo SITL.

## Project Progress

### Completed

- Nonlinear 6-DOF quadrotor dynamics with 12 states.
- Open-loop validation: hover equilibrium, free fall, and roll perturbation.
- Linearization around the hover equilibrium.
- Controllability analysis of the linearized state-space model.
- Continuous-time LQR controller design.
- Closed-loop LQR stabilization tested on the nonlinear quadrotor model.
- Visualization of roll, lateral position, and altitude responses.

### In Progress / Planned

- Cascaded PID attitude and position control.
- Quantitative PID vs. LQR performance comparison.
- MPC controller implementation.
- EKF-based state estimation.
- Robustness analysis under disturbances and parameter uncertainty.
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