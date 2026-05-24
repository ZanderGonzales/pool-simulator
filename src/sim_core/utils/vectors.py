"""Small 2D vector helpers over NumPy."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

Vec2 = NDArray[np.float64]


def vec2(x: float, y: float) -> Vec2:
    """Create a 2D float64 vector."""
    return np.array([x, y], dtype=np.float64)


def norm(v: Vec2) -> float:
    """Euclidean norm of a 2D vector."""
    return float(np.linalg.norm(v))


def normalize(v: Vec2) -> Vec2:
    """Return unit vector; zero vector if input has zero length."""
    n = norm(v)
    if n == 0.0:
        return vec2(0.0, 0.0)
    return v / n


def dot(a: Vec2, b: Vec2) -> float:
    """Dot product of two 2D vectors."""
    return float(np.dot(a, b))
