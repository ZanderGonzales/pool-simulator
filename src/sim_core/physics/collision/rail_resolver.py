"""Ball-cushion separation and velocity reflection."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from sim_core.physics.ball import Ball
from sim_core.physics.collision.rail_detector import BallRailContact
from sim_core.physics.table import TableConfig
from sim_core.utils.vectors import dot

Vec2 = NDArray[np.float64]


def separate_ball_from_cushion(ball: Ball, normal: Vec2, penetration: float) -> None:
    """Push a penetrating ball into the playable area along the cushion normal."""
    if penetration <= 0.0:
        return
    ball.position = ball.position + normal * penetration


def resolve_ball_rail_contact(
    ball: Ball,
    contact: BallRailContact,
    table: TableConfig,
) -> bool:
    """
    Separate overlap, then reflect incoming normal velocity into the cushion.

    Positional correction runs whenever penetration > 0 (same pattern as Phase 2).
    Velocity reflection runs only when v_n = dot(v, n) < 0. Returns True if velocity
    changed.
    """
    normal = contact.normal
    if contact.penetration > 0.0:
        separate_ball_from_cushion(ball, normal, contact.penetration)

    normal_speed = dot(ball.velocity, normal)
    if normal_speed >= 0.0:
        return False

    normal_velocity = normal * normal_speed
    tangent_velocity = ball.velocity - normal_velocity
    e = table.coefficient_of_restitution
    damping = table.cushion_tangential_damping
    ball.velocity = (-e * normal_velocity) + ((1.0 - damping) * tangent_velocity)
    return True


def resolve_ball_rail_contacts(
    balls: list[Ball],
    contacts: list[BallRailContact],
    table: TableConfig,
) -> int:
    """Resolve each rail contact once. Returns the number of reflections applied."""
    applied = 0
    sorted_contacts = sorted(contacts, key=lambda contact: (contact.index_a, contact.index_b))
    for contact in sorted_contacts:
        ball = balls[contact.index_a]
        if resolve_ball_rail_contact(ball, contact, table):
            applied += 1
    return applied
