"""Cue shot parameters (initial conditions, not a full cue impact model)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from sim_core.utils.constants import BALL_RADIUS_M
from sim_core.utils.vectors import norm, normalize, vec2, vec3

Vec2 = NDArray[np.float64]
Vec3 = NDArray[np.float64]


@dataclass(frozen=True)
class ShotParams:
    """Translational speed and optional initial angular velocity."""

    speed: float
    omega: float = 0.0
    angular_velocity: Vec3 | None = None

    def __post_init__(self) -> None:
        if self.speed < 0:
            raise ValueError("speed must be non-negative")
        if self.angular_velocity is not None:
            av = np.asarray(self.angular_velocity, dtype=np.float64)
            if av.shape != (3,):
                raise ValueError("angular_velocity must be a 3D vector")
            object.__setattr__(self, "angular_velocity", av.copy())

    def resolved_angular_velocity(self) -> Vec3:
        """Return the 3D angular velocity for this shot."""
        if self.angular_velocity is not None:
            return np.asarray(self.angular_velocity, dtype=np.float64).copy()
        return vec3(0.0, 0.0, self.omega)


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

    def to_angular_velocity(self, *, radius: float = BALL_RADIUS_M) -> Vec3:
        """
        Map hit offsets to angular velocity.

        Parallel offset: follow/draw about the vertical axis (omega_z).
        Perpendicular offset: side spin about the aim direction (omega_x, omega_y).
        """
        scale = self.speed / max(radius, 1e-9)
        aim = self.aim_direction
        omega_z = scale * self.hit_offset_parallel
        omega_side = scale * self.hit_offset_perpendicular
        return vec3(aim[0] * omega_side, aim[1] * omega_side, omega_z)

    def to_shot_params(self, *, radius: float = BALL_RADIUS_M) -> ShotParams:
        """Convert strike offsets into speed and full angular velocity."""
        angular_velocity = self.to_angular_velocity(radius=radius)
        return ShotParams(
            speed=self.speed,
            omega=float(angular_velocity[2]),
            angular_velocity=angular_velocity,
        )

    def velocity_vector(self) -> Vec2:
        """Return initial cue-ball velocity vector in table coordinates."""
        return self.aim_direction * self.speed
