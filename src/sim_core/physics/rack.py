"""15-ball triangle rack layout and break-shot setup."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from sim_core.physics.ball import Ball
from sim_core.physics.shot import CueStrike, ShotParams
from sim_core.utils.constants import (
    BALL_RADIUS_M,
    DEFAULT_FOOT_SPOT_X_FRACTION,
    DEFAULT_HEAD_SPOT_X_FRACTION,
    ROW_SPACING_M,
    TABLE_HEIGHT_M,
    TABLE_WIDTH_M,
)
from sim_core.utils.vectors import norm, vec2

Vec2 = NDArray[np.float64]

RACK_BALL_COUNT = 15
ROW_COUNT = 5


@dataclass(frozen=True)
class RackLayout:
    """A racked set of object balls with metadata for the apex ball."""

    balls: tuple[Ball, ...]
    apex_index: int

    def __post_init__(self) -> None:
        if len(self.balls) != RACK_BALL_COUNT:
            raise ValueError(f"rack must contain exactly {RACK_BALL_COUNT} balls")
        validate_no_overlaps(list(self.balls))


@dataclass(frozen=True)
class BreakSetup:
    """Cue ball plus a full rack configured for a break shot."""

    balls: tuple[Ball, ...]
    cue_ball_index: int
    apex_index: int

    def __post_init__(self) -> None:
        if len(self.balls) != RACK_BALL_COUNT + 1:
            raise ValueError("break setup must contain cue ball plus 15 object balls")
        validate_no_overlaps(list(self.balls))


def default_foot_spot(
    table_width: float = TABLE_WIDTH_M,
    table_height: float = TABLE_HEIGHT_M,
) -> Vec2:
    """Return the default foot-spot position on the long table axis."""
    return vec2(table_width * DEFAULT_FOOT_SPOT_X_FRACTION, table_height * 0.5)


def default_head_spot(
    table_width: float = TABLE_WIDTH_M,
    table_height: float = TABLE_HEIGHT_M,
) -> Vec2:
    """Return the default head-spot position on the long table axis."""
    return vec2(table_width * DEFAULT_HEAD_SPOT_X_FRACTION, table_height * 0.5)


def validate_no_overlaps(
    balls: list[Ball],
    *,
    tolerance: float = 1e-9,
) -> None:
    """Raise ValueError if any active pair of balls overlaps."""
    for i in range(len(balls)):
        ball_a = balls[i]
        if not ball_a.active:
            continue
        for j in range(i + 1, len(balls)):
            ball_b = balls[j]
            if not ball_b.active:
                continue
            min_distance = ball_a.radius + ball_b.radius
            distance = norm(ball_b.position - ball_a.position)
            if distance < min_distance - tolerance:
                raise ValueError(
                    f"balls {ball_a.id} and {ball_b.id} overlap: "
                    f"distance={distance:.6f}, min={min_distance:.6f}"
                )


def triangle_rack(
    *,
    foot_spot: Vec2 | None = None,
    table_width: float = TABLE_WIDTH_M,
    table_height: float = TABLE_HEIGHT_M,
    radius: float = BALL_RADIUS_M,
    start_id: int = 0,
) -> RackLayout:
    """
    Build a 15-ball triangle with the apex at foot_spot facing the head side.

    Rows extend from the apex toward the foot rail (-x). Balls in each row are
    spaced 2*radius apart center-to-center.
    """
    if foot_spot is None:
        foot_spot = default_foot_spot(table_width, table_height)

    balls: list[Ball] = []
    apex_index = 0

    for row in range(ROW_COUNT):
        row_ball_count = row + 1
        x = foot_spot[0] - row * ROW_SPACING_M * (radius / BALL_RADIUS_M)
        for col in range(row_ball_count):
            y = foot_spot[1] + (col - 0.5 * (row_ball_count - 1)) * 2.0 * radius
            ball_id = start_id + len(balls)
            balls.append(
                Ball(
                    id=ball_id,
                    position=vec2(x, y),
                    velocity=vec2(0.0, 0.0),
                    radius=radius,
                )
            )

    return RackLayout(balls=tuple(balls), apex_index=apex_index)


def place_cue_ball(
    rack: RackLayout,
    *,
    head_spot: Vec2 | None = None,
    table_width: float = TABLE_WIDTH_M,
    table_height: float = TABLE_HEIGHT_M,
    gap: float = 0.05,
    cue_ball_id: int = 15,
    radius: float = BALL_RADIUS_M,
) -> Ball:
    """Place the cue ball on the head side, aligned with the rack apex."""
    if head_spot is None:
        head_spot = default_head_spot(table_width, table_height)

    apex = rack.balls[rack.apex_index]
    separation = 2.0 * radius + gap
    cue_x = apex.position[0] + separation
    cue_y = apex.position[1]

    if cue_x > head_spot[0]:
        cue_x = head_spot[0]

    return Ball(
        id=cue_ball_id,
        position=vec2(cue_x, cue_y),
        velocity=vec2(0.0, 0.0),
        radius=radius,
    )


def create_break_setup(
    *,
    break_speed: float = 3.0,
    shot: ShotParams | None = None,
    cue_strike: CueStrike | None = None,
    foot_spot: Vec2 | None = None,
    head_spot: Vec2 | None = None,
    table_width: float = TABLE_WIDTH_M,
    table_height: float = TABLE_HEIGHT_M,
    cue_gap: float = 0.05,
    radius: float = BALL_RADIUS_M,
) -> BreakSetup:
    """Create a cue ball plus triangle rack with the cue moving toward the apex."""
    velocity = vec2(-break_speed, 0.0)
    if shot is not None:
        break_speed = shot.speed
        velocity = vec2(-break_speed, 0.0)
    if cue_strike is not None:
        shot = cue_strike.to_shot_params(radius=radius)
        velocity = cue_strike.velocity_vector()
    rack = triangle_rack(
        foot_spot=foot_spot,
        table_width=table_width,
        table_height=table_height,
        radius=radius,
    )
    cue_ball = place_cue_ball(
        rack,
        head_spot=head_spot,
        table_width=table_width,
        table_height=table_height,
        gap=cue_gap,
        radius=radius,
    )
    cue_ball.velocity = velocity
    if shot is not None:
        cue_ball.omega = shot.omega

    balls = list(rack.balls) + [cue_ball]
    validate_no_overlaps(balls)

    return BreakSetup(
        balls=tuple(balls),
        cue_ball_index=len(balls) - 1,
        apex_index=rack.apex_index,
    )
