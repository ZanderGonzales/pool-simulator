"""Small 2D vector helpers over NumPy."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

Vec2 = NDArray[np.float64]
Vec3 = NDArray[np.float64]


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


def vec3(x: float, y: float, z: float) -> Vec3:
    """Create a 3D float64 vector."""
    return np.array([x, y, z], dtype=np.float64)


def norm3(v: Vec3) -> float:
    """Euclidean norm of a 3D vector."""
    return float(np.linalg.norm(v))


def cross3(a: Vec3, b: Vec3) -> Vec3:
    """Cross product of two 3D vectors."""
    return np.cross(a, b)


def vec2_to_vec3(v: Vec2, z: float = 0.0) -> Vec3:
    """Promote a 2D vector to 3D."""
    return vec3(float(v[0]), float(v[1]), z)
