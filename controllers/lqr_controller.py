
import numpy as np
import control

from models.linearized import hover_linear_model


def design_lqr(params):
    """
    Design a continuous-time LQR controller
    around the quadrotor hover equilibrium.
    """

    A, B = hover_linear_model(params)

    # State weighting matrix
    Q = np.diag([
        10, 10, 20,      # X, Y, Z
        5, 5, 10,        # vx, vy, vz
        100, 100, 20,   # phi, theta, psi
        10, 10, 5       # p, q, r
    ])

    # Control effort weighting matrix
    R = np.diag([
        1, 1, 1, 1
    ])

    # Solve the LQR problem
    K, P, poles = control.lqr(A, B, Q, R)

    return np.asarray(K), poles