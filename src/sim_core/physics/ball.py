from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from sim_core.utils.constants import BALL_MASS_KG, BALL_RADIUS_M
from sim_core.utils.vectors import norm, vec2

Vec2 = NDArray[np.float64]


@dataclass
class Ball:
    """A billiard ball with 2D position, velocity, and spin about the vertical axis."""

    id: int
    position: Vec2
    velocity: Vec2
    radius: float = BALL_RADIUS_M
    mass: float = BALL_MASS_KG
    active: bool = True
    omega: float = 0.0

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=np.float64).copy()
        self.velocity = np.asarray(self.velocity, dtype=np.float64).copy()
        if self.position.shape != (2,) or self.velocity.shape != (2,):
            raise ValueError("position and velocity must be 2D vectors")

    @property
    def speed(self) -> float:
        return norm(self.velocity)

    def is_moving(
        self,
        velocity_stop_threshold: float,
        omega_stop_threshold: float | None = None,
    ) -> bool:
        if not self.active:
            return False
        if self.speed >= velocity_stop_threshold:
            return True
        if omega_stop_threshold is not None and abs(self.omega) >= omega_stop_threshold:
            return True
        return False

    def stop(self) -> None:
        self.velocity = vec2(0.0, 0.0)
        self.omega = 0.0

    def copy(self) -> Ball:
        """Return a shallow copy of position, velocity, and spin state."""
        return Ball(
            id=self.id,
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            radius=self.radius,
            mass=self.mass,
            active=self.active,
            omega=self.omega,
        )
