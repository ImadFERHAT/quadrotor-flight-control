
import numpy as np


def hover_linear_model(params):
    """
    Linearized 12-state quadrotor model around hover.

    State:
    [X, Y, Z, vx, vy, vz, phi, theta, psi, p, q, r]

    Input deviations:
    [delta_T, tau_phi, tau_theta, tau_psi]
    """
    m = params["m"]
    g = params["g"]
    Jx, Jy, Jz = np.diag(params["J"])

    A = np.zeros((12, 12))
    B = np.zeros((12, 4))

    # Position derivatives
    A[0, 3] = 1
    A[1, 4] = 1
    A[2, 5] = 1

    # Translational acceleration
    A[3, 7] = g
    A[4, 6] = -g

    # Euler-angle kinematics
    A[6, 9] = 1
    A[7, 10] = 1
    A[8, 11] = 1

    # Control inputs
    B[5, 0] = 1 / m
    B[9, 1] = 1 / Jx
    B[10, 2] = 1 / Jy
    B[11, 3] = 1 / Jz

    return A, B
