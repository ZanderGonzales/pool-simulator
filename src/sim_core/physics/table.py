from __future__ import annotations

from dataclasses import dataclass

from sim_core.utils.constants import (
    DEFAULT_ROLLING_RESISTANCE_COEFFICIENT,
    DEFAULT_VELOCITY_STOP_THRESHOLD,
    TABLE_HEIGHT_M,
    TABLE_WIDTH_M,
)


@dataclass(frozen=True)
class TableConfig:
    """Table geometry and rolling-resistance parameters."""

    width: float = TABLE_WIDTH_M
    height: float = TABLE_HEIGHT_M
    rolling_resistance_coefficient: float = DEFAULT_ROLLING_RESISTANCE_COEFFICIENT
    velocity_stop_threshold: float = DEFAULT_VELOCITY_STOP_THRESHOLD

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("table width and height must be positive")
        if self.rolling_resistance_coefficient < 0:
            raise ValueError("rolling_resistance_coefficient must be non-negative")
        if self.velocity_stop_threshold <= 0:
            raise ValueError("velocity_stop_threshold must be positive")
