from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from sim_core.utils.constants import BALL_MASS_KG, BALL_RADIUS_M
from sim_core.utils.vectors import norm, norm3, vec2, vec3

Vec2 = NDArray[np.float64]
Vec3 = NDArray[np.float64]


@dataclass(init=False)
class Ball:
    """A billiard ball with 2D center motion and 3D angular velocity (z-up)."""

    id: int
    position: Vec2
    velocity: Vec2
    radius: float
    mass: float
    active: bool
    angular_velocity: Vec3

    def __init__(
        self,
        id: int,
        position: Vec2,
        velocity: Vec2,
        radius: float = BALL_RADIUS_M,
        mass: float = BALL_MASS_KG,
        active: bool = True,
        angular_velocity: Vec3 | None = None,
        omega: float | None = None,
    ) -> None:
        self.id = id
        self.position = np.asarray(position, dtype=np.float64).copy()
        self.velocity = np.asarray(velocity, dtype=np.float64).copy()
        self.radius = radius
        self.mass = mass
        self.active = active
        if angular_velocity is None:
            self.angular_velocity = vec3(0.0, 0.0, 0.0)
        else:
            self.angular_velocity = np.asarray(angular_velocity, dtype=np.float64).copy()
        if self.position.shape != (2,) or self.velocity.shape != (2,):
            raise ValueError("position and velocity must be 2D vectors")
        if self.angular_velocity.shape != (3,):
            raise ValueError("angular_velocity must be a 3D vector")
        if omega is not None:
            self.angular_velocity[2] = float(omega)

    @property
    def omega(self) -> float:
        """Scalar top-spin component (rad/s) about the vertical axis."""
        return float(self.angular_velocity[2])

    @omega.setter
    def omega(self, value: float) -> None:
        self.angular_velocity[2] = float(value)

    @property
    def spin_speed(self) -> float:
        """Magnitude of angular velocity (rad/s)."""
        return norm3(self.angular_velocity)

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
        if omega_stop_threshold is not None and self.spin_speed >= omega_stop_threshold:
            return True
        return False

    def stop(self) -> None:
        self.velocity = vec2(0.0, 0.0)
        self.angular_velocity = vec3(0.0, 0.0, 0.0)

    def copy(self) -> Ball:
        """Return a shallow copy of position, velocity, and spin state."""
        return Ball(
            id=self.id,
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            radius=self.radius,
            mass=self.mass,
            active=self.active,
            angular_velocity=self.angular_velocity.copy(),
        )
