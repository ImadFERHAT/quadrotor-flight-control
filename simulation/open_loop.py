
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Allow imports from the project root
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from models.quadrotor import quadrotor_dynamics


params = {
    "m": 1.0,
    "g": 9.81,
    "J": np.diag([0.01, 0.01, 0.02])
}

m = params["m"]
g = params["g"]

t_eval = np.linspace(0, 2, 201)

# Initial state: 10 m altitude, stationary and level
x0 = np.zeros(12)
x0[2] = 10.0


def simulate(name, initial_state, u):
    sol = solve_ivp(
        fun=lambda t, x: quadrotor_dynamics(t, x, u, params),
        t_span=(t_eval[0], t_eval[-1]),
        y0=initial_state,
        t_eval=t_eval,
        rtol=1e-9,
        atol=1e-11
    )

    if not sol.success:
        raise RuntimeError(sol.message)

    return sol


# Test 1: Hover
u_hover = np.array([m * g, 0, 0, 0])
hover = simulate("hover", x0, u_hover)

# Test 2: Free fall
u_fall = np.zeros(4)
free_fall = simulate("free_fall", x0, u_fall)

# Test 3: Initial roll perturbation of 5 degrees
x_perturbed = x0.copy()
x_perturbed[6] = np.deg2rad(5)

perturbed = simulate(
    "perturbed", x_perturbed, u_hover
)

# Basic validation tests
assert np.allclose(hover.y[2], 10.0, atol=1e-7)
assert np.allclose(hover.y[3:6], 0.0, atol=1e-7)

expected_fall = 10 - 0.5 * g * t_eval**2
assert np.allclose(free_fall.y[2], expected_fall, atol=1e-7)

assert np.allclose(
    perturbed.y[6],
    np.deg2rad(5),
    atol=1e-7
)

print("All three open-loop tests passed!")

# Plot altitude
plt.figure(figsize=(9, 5))
plt.plot(hover.t, hover.y[2], label="Hover")
plt.plot(free_fall.t, free_fall.y[2], label="Free fall")
plt.plot(
    perturbed.t,
    perturbed.y[2],
    label="5 deg roll perturbation"
)

plt.xlabel("Time [s]")
plt.ylabel("Altitude Z [m]")
plt.title("Quadrotor Open-Loop Altitude")
plt.legend()
plt.grid(True)
plt.tight_layout()

results = ROOT / "results"
results.mkdir(exist_ok=True)
plt.savefig(results / "open_loop_altitude.png", dpi=160)

# Plot horizontal displacement
plt.figure(figsize=(9, 5))
plt.plot(
    perturbed.t,
    perturbed.y[1],
    label="Y displacement"
)
plt.xlabel("Time [s]")
plt.ylabel("Y position [m]")
plt.title("Horizontal Drift from Roll Perturbation")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(results / "open_loop_drift.png", dpi=160)

plt.show()

plt.figure(figsize=(9, 5))

plt.plot(
    hover.t,
    hover.y[2],
    label="Hover",
    linewidth=2
)

plt.plot(
    perturbed.t,
    perturbed.y[2],
    label="5° roll perturbation",
    linewidth=2
)

plt.xlabel("Time [s]")
plt.ylabel("Altitude Z [m]")
plt.title("Effect of Roll Perturbation on Altitude")

plt.ylim(9.90, 10.01)
plt.grid(True)
plt.legend()
plt.tight_layout()

plt.savefig(
    results / "roll_altitude_zoom.png",
    dpi=160
)

plt.show()

print(f"Hover final altitude: {hover.y[2, -1]:.6f} m")
print(f"Perturbed final altitude: {perturbed.y[2, -1]:.6f} m")
print(f"Altitude loss: {10 - perturbed.y[2, -1]:.6f} m")