
import numpy as np


def rotation_matrix(phi, theta, psi):
    """Body-to-world rotation matrix, ZYX convention."""
    cphi, sphi = np.cos(phi), np.sin(phi)
    cth, sth = np.cos(theta), np.sin(theta)
    cpsi, spsi = np.cos(psi), np.sin(psi)

    Rx = np.array([
        [1, 0, 0],
        [0, cphi, -sphi],
        [0, sphi, cphi]
    ])

    Ry = np.array([
        [cth, 0, sth],
        [0, 1, 0],
        [-sth, 0, cth]
    ])

    Rz = np.array([
        [cpsi, -spsi, 0],
        [spsi, cpsi, 0],
        [0, 0, 1]
    ])

    return Rz @ Ry @ Rx


def quadrotor_dynamics(t, x, u, params):
    """
    Nonlinear 12-state quadrotor dynamics.

    x = [X, Y, Z, vx, vy, vz, phi, theta, psi, p, q, r]
    u = [T, tau_phi, tau_theta, tau_psi]

    World frame: Z upward.
    Body frame: Z upward.
    """
    m = params["m"]
    g = params["g"]
    J = params["J"]

    position = x[0:3]
    velocity = x[3:6]
    phi, theta, psi = x[6:9]
    omega = x[9:12]

    T = u[0]
    tau = np.asarray(u[1:4])

    # Translational dynamics
    R = rotation_matrix(phi, theta, psi)

    gravity = np.array([0.0, 0.0, -g])
    thrust_body = np.array([0.0, 0.0, T])

    acceleration = gravity + R @ thrust_body / m

    # Euler-angle kinematics
    p, q, r = omega

    if abs(np.cos(theta)) < 1e-6:
        raise ValueError("Euler-angle singularity near pitch = +/-90 deg")

    euler_rates = np.array([
        p + np.sin(phi) * np.tan(theta) * q
          + np.cos(phi) * np.tan(theta) * r,
        np.cos(phi) * q - np.sin(phi) * r,
        np.sin(phi) / np.cos(theta) * q
          + np.cos(phi) / np.cos(theta) * r
    ])

    # Rotational dynamics: J*w_dot = tau - w x (J*w)
    omega_dot = np.linalg.solve(
        J, tau - np.cross(omega, J @ omega)
    )

    return np.concatenate([
        velocity,
        acceleration,
        euler_rates,
        omega_dot
    ])
