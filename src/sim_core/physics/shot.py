"""Cue shot parameters (initial conditions, not a full cue impact model)."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ShotParams:
    """Translational speed and english (spin about the vertical axis)."""

    speed: float
    omega: float = 0.0

    def __post_init__(self) -> None:
        if self.speed < 0:
            raise ValueError("speed must be non-negative")
