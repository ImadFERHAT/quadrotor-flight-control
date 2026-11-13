
import sys
from pathlib import Path

import numpy as np
import control

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from models.linearized import hover_linear_model


params = {
    "m": 1.0,
    "g": 9.81,
    "J": np.diag([0.01, 0.01, 0.02])
}

A, B = hover_linear_model(params)

Wc = control.ctrb(A, B)
rank = np.linalg.matrix_rank(Wc)

print("A shape:", A.shape)
print("B shape:", B.shape)
print("Controllability rank:", rank)

assert rank == 12, "System is not controllable"

print("The linearized system is controllable.")
