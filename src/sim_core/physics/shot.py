"""Cue shot parameters (initial conditions, not a full cue impact model)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from sim_core.utils.constants import BALL_RADIUS_M
from sim_core.utils.vectors import norm, normalize, vec2

Vec2 = NDArray[np.float64]


@dataclass(frozen=True)
class ShotParams:
    """Translational speed and english (spin about the vertical axis)."""

    speed: float
    omega: float = 0.0

    def __post_init__(self) -> None:
        if self.speed < 0:
            raise ValueError("speed must be non-negative")


@dataclass(frozen=True)
class CueStrike:
    """Cue strike as speed, aim direction, and normalized hit offsets."""

    speed: float
    aim_direction: Vec2
    hit_offset_parallel: float = 0.0
    hit_offset_perpendicular: float = 0.0

    def __post_init__(self) -> None:
        direction = np.asarray(self.aim_direction, dtype=np.float64).copy()
        if direction.shape != (2,):
            raise ValueError("aim_direction must be a 2D vector")
        if self.speed < 0:
            raise ValueError("speed must be non-negative")
        if norm(direction) == 0.0:
            raise ValueError("aim_direction must be non-zero")
        if abs(self.hit_offset_parallel) > 1.0 or abs(self.hit_offset_perpendicular) > 1.0:
            raise ValueError("hit offsets must be in [-1, 1]")
        object.__setattr__(self, "aim_direction", normalize(direction))

    def to_shot_params(self, *, radius: float = BALL_RADIUS_M) -> ShotParams:
        """Convert strike offsets into translational speed and scalar spin."""
        omega_scale = self.speed / max(radius, 1e-9)
        omega = omega_scale * (
            self.hit_offset_parallel + 0.25 * self.hit_offset_perpendicular
        )
        return ShotParams(speed=self.speed, omega=float(omega))

    def velocity_vector(self) -> Vec2:
        """Return initial cue-ball velocity vector in table coordinates."""
        return self.aim_direction * self.speed
