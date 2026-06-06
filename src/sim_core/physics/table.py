from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from sim_core.utils.constants import (
    DEFAULT_BALL_BALL_FRICTION,
    DEFAULT_COEFFICIENT_OF_RESTITUTION,
    DEFAULT_OMEGA_STOP_THRESHOLD,
    DEFAULT_POCKET_RADIUS_M,
    DEFAULT_RAIL_FRICTION,
    DEFAULT_ROLLING_RESISTANCE_COEFFICIENT,
    DEFAULT_SLIDING_FRICTION_COEFFICIENT,
    DEFAULT_SPIN_DECAY_RATE,
    DEFAULT_SWERVE_COEFFICIENT,
    DEFAULT_VELOCITY_STOP_THRESHOLD,
    SLIDING_SPEED_THRESHOLD,
    TABLE_HEIGHT_M,
    TABLE_WIDTH_M,
)
from sim_core.utils.vectors import vec2

Vec2 = NDArray[np.float64]


@dataclass(frozen=True)
class CushionSegment:
    """A straight cushion segment with a normal pointing into the playable area."""

    start: Vec2
    end: Vec2
    normal: Vec2

    def __post_init__(self) -> None:
        start = np.asarray(self.start, dtype=np.float64).copy()
        end = np.asarray(self.end, dtype=np.float64).copy()
        normal = np.asarray(self.normal, dtype=np.float64).copy()
        if start.shape != (2,) or end.shape != (2,) or normal.shape != (2,):
            raise ValueError("cushion start, end, and normal must be 2D vectors")
        length = float(np.linalg.norm(end - start))
        normal_length = float(np.linalg.norm(normal))
        if length <= 0.0:
            raise ValueError("cushion segment length must be positive")
        if normal_length <= 0.0:
            raise ValueError("cushion normal must be non-zero")
        object.__setattr__(self, "start", start)
        object.__setattr__(self, "end", end)
        object.__setattr__(self, "normal", normal / normal_length)


def default_cushions(width: float, height: float) -> tuple[CushionSegment, ...]:
    """Return rectangular table cushions around the playable surface."""
    return (
        CushionSegment(
            start=vec2(0.0, 0.0),
            end=vec2(width, 0.0),
            normal=vec2(0.0, 1.0),
        ),
        CushionSegment(
            start=vec2(width, 0.0), end=vec2(width, height), normal=vec2(-1.0, 0.0)
        ),
        CushionSegment(
            start=vec2(width, height), end=vec2(0.0, height), normal=vec2(0.0, -1.0)
        ),
        CushionSegment(
            start=vec2(0.0, height), end=vec2(0.0, 0.0), normal=vec2(1.0, 0.0)
        ),
    )


@dataclass(frozen=True)
class Pocket:
    """Simple circular pocket capture region."""

    center: Vec2
    radius: float = DEFAULT_POCKET_RADIUS_M

    def __post_init__(self) -> None:
        center = np.asarray(self.center, dtype=np.float64).copy()
        if center.shape != (2,):
            raise ValueError("pocket center must be a 2D vector")
        if self.radius <= 0.0:
            raise ValueError("pocket radius must be positive")
        object.__setattr__(self, "center", center)


def default_pockets(width: float, height: float, radius: float) -> tuple[Pocket, ...]:
    """Return six regulation-style pockets: 4 corners and 2 side pockets."""
    return (
        Pocket(center=vec2(0.0, 0.0), radius=radius),
        Pocket(center=vec2(width, 0.0), radius=radius),
        Pocket(center=vec2(width, height), radius=radius),
        Pocket(center=vec2(0.0, height), radius=radius),
        Pocket(center=vec2(0.5 * width, 0.0), radius=radius),
        Pocket(center=vec2(0.5 * width, height), radius=radius),
    )


@dataclass(frozen=True)
class TableConfig:
    """Table geometry, friction, and spin parameters."""

    width: float = TABLE_WIDTH_M
    height: float = TABLE_HEIGHT_M
    rolling_resistance_coefficient: float = DEFAULT_ROLLING_RESISTANCE_COEFFICIENT
    velocity_stop_threshold: float = DEFAULT_VELOCITY_STOP_THRESHOLD
    coefficient_of_restitution: float = DEFAULT_COEFFICIENT_OF_RESTITUTION
    cushion_tangential_damping: float = 0.0
    sliding_friction_coefficient: float = DEFAULT_SLIDING_FRICTION_COEFFICIENT
    ball_ball_friction: float = DEFAULT_BALL_BALL_FRICTION
    rail_friction: float = DEFAULT_RAIL_FRICTION
    spin_decay_rate: float = DEFAULT_SPIN_DECAY_RATE
    omega_stop_threshold: float = DEFAULT_OMEGA_STOP_THRESHOLD
    sliding_speed_threshold: float = SLIDING_SPEED_THRESHOLD
    swerve_coefficient: float = DEFAULT_SWERVE_COEFFICIENT
    pocket_radius: float = DEFAULT_POCKET_RADIUS_M
    cushions: tuple[CushionSegment, ...] = field(default_factory=tuple)
    pockets: tuple[Pocket, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("table width and height must be positive")
        if self.rolling_resistance_coefficient < 0:
            raise ValueError("rolling_resistance_coefficient must be non-negative")
        if self.velocity_stop_threshold <= 0:
            raise ValueError("velocity_stop_threshold must be positive")
        if not 0.0 <= self.coefficient_of_restitution <= 1.0:
            raise ValueError("coefficient_of_restitution must be in [0, 1]")
        if not 0.0 <= self.cushion_tangential_damping <= 1.0:
            raise ValueError("cushion_tangential_damping must be in [0, 1]")
        if self.sliding_friction_coefficient < 0:
            raise ValueError("sliding_friction_coefficient must be non-negative")
        if self.ball_ball_friction < 0:
            raise ValueError("ball_ball_friction must be non-negative")
        if self.rail_friction < 0:
            raise ValueError("rail_friction must be non-negative")
        if self.spin_decay_rate < 0:
            raise ValueError("spin_decay_rate must be non-negative")
        if self.omega_stop_threshold <= 0:
            raise ValueError("omega_stop_threshold must be positive")
        if self.sliding_speed_threshold <= 0:
            raise ValueError("sliding_speed_threshold must be positive")
        if self.swerve_coefficient < 0:
            raise ValueError("swerve_coefficient must be non-negative")
        if self.pocket_radius <= 0:
            raise ValueError("pocket_radius must be positive")
        if not self.cushions:
            object.__setattr__(self, "cushions", default_cushions(self.width, self.height))
        if not self.pockets:
            object.__setattr__(
                self,
                "pockets",
                default_pockets(self.width, self.height, self.pocket_radius),
            )
