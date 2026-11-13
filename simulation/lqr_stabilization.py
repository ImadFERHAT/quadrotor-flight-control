
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from models.quadrotor import quadrotor_dynamics
from controllers.lqr_controller import design_lqr


# Quadrotor parameters
params = {
    "m": 1.0,
    "g": 9.81,
    "J": np.diag([0.01, 0.01, 0.02])
}

# Design the LQR controller
K, poles = design_lqr(params)

# Desired hover state: position (0, 0, 10)
x_ref = np.zeros(12)
x_ref[2] = 10.0

# Hover input
u_hover = np.array([
    params["m"] * params["g"],
    0.0, 0.0, 0.0
])

# Initial perturbation: 5-degree roll
x_initial = x_ref.copy()
x_initial[6] = np.deg2rad(5)


def closed_loop(t, x):
    # State error
    error = x - x_ref

    # LQR feedback
    u = u_hover - K @ error

    # Original nonlinear dynamics
    return quadrotor_dynamics(t, x, u, params)


# Simulation
t_eval = np.linspace(0, 10, 1001)

sol = solve_ivp(
    closed_loop,
    (0, 10),
    x_initial,
    t_eval=t_eval,
    rtol=1e-8,
    atol=1e-10
)

if not sol.success:
    raise RuntimeError(sol.message)

# Stability check
print("Closed-loop eigenvalues:")
print(poles)

assert np.all(np.real(poles) < 0), (
    "The linear closed-loop system is not stable."
)

print(
    "Final position error:",
    np.linalg.norm(sol.y[:3, -1] - x_ref[:3])
)

print(
    "Final roll angle [deg]:",
    np.rad2deg(sol.y[6, -1])
)

# Plot results
fig, axes = plt.subplots(3, 1, figsize=(9, 9))

axes[0].plot(sol.t, np.rad2deg(sol.y[6]))
axes[0].axhline(0, color="black", linestyle="--")
axes[0].set_ylabel("Roll [deg]")
axes[0].grid(True)

axes[1].plot(sol.t, sol.y[1])
axes[1].axhline(0, color="black", linestyle="--")
axes[1].set_ylabel("Y position [m]")
axes[1].grid(True)

axes[2].plot(sol.t, sol.y[2])
axes[2].axhline(10, color="black", linestyle="--")
axes[2].set_ylabel("Altitude [m]")
axes[2].set_xlabel("Time [s]")
axes[2].grid(True)

fig.suptitle("Nonlinear Quadrotor Stabilization with LQR")
fig.tight_layout()

results = ROOT / "results"
results.mkdir(exist_ok=True)

fig.savefig(
    results / "lqr_stabilization.png",
    dpi=160
)

plt.show()