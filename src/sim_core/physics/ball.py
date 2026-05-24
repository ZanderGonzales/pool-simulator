from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from sim_core.utils.constants import BALL_MASS_KG, BALL_RADIUS_M
from sim_core.utils.vectors import norm, vec2

Vec2 = NDArray[np.float64]


@dataclass
class Ball:
    """A billiard ball with 2D position and velocity."""

    id: int
    position: Vec2
    velocity: Vec2
    radius: float = BALL_RADIUS_M
    mass: float = BALL_MASS_KG
    active: bool = True

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=np.float64).copy()
        self.velocity = np.asarray(self.velocity, dtype=np.float64).copy()
        if self.position.shape != (2,) or self.velocity.shape != (2,):
            raise ValueError("position and velocity must be 2D vectors")

    @property
    def speed(self) -> float:
        return norm(self.velocity)

    def is_moving(self, velocity_stop_threshold: float) -> bool:
        return self.active and self.speed >= velocity_stop_threshold

    def stop(self) -> None:
        self.velocity = vec2(0.0, 0.0)
