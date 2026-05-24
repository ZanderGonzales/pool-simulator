"""Impulse-based ball-ball collision resolution (frictionless contact)."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from sim_core.physics.ball import Ball
from sim_core.physics.collision.detector import BallBallContact
from sim_core.physics.table import TableConfig
from sim_core.utils.vectors import dot

Vec2 = NDArray[np.float64]


def separate_balls(ball_a: Ball, ball_b: Ball, normal: Vec2, penetration: float) -> None:
    """Push overlapping balls apart along the contact normal (equal split)."""
    if penetration <= 0.0:
        return
    half = 0.5 * penetration
    ball_a.position = ball_a.position - normal * half
    ball_b.position = ball_b.position + normal * half


def resolve_ball_ball_contact(
    ball_a: Ball,
    ball_b: Ball,
    contact: BallBallContact,
    table: TableConfig,
) -> bool:
    """
    Apply a normal impulse for equal-mass spheres with restitution e.

    Normal n points from ball_a to ball_b. Tangential friction is deferred to
    Phase 5. Returns True if an impulse was applied.
    """
    normal = contact.normal
    applied_impulse = False

    v_rel_n = dot(ball_b.velocity - ball_a.velocity, normal)
    if v_rel_n < 0.0:
        e = table.coefficient_of_restitution
        inv_mass_sum = (1.0 / ball_a.mass) + (1.0 / ball_b.mass)
        impulse = -(1.0 + e) * v_rel_n / inv_mass_sum

        ball_a.velocity = ball_a.velocity - (impulse / ball_a.mass) * normal
        ball_b.velocity = ball_b.velocity + (impulse / ball_b.mass) * normal
        applied_impulse = True

    if contact.penetration > 0.0:
        separate_balls(ball_a, ball_b, normal, contact.penetration)

    return applied_impulse


def resolve_ball_ball_contacts(
    balls: list[Ball],
    contacts: list[BallBallContact],
    table: TableConfig,
) -> int:
    """Resolve each contact once. Returns the number of impulses applied."""
    applied = 0
    sorted_contacts = sorted(contacts, key=lambda contact: (contact.index_a, contact.index_b))
    for contact in sorted_contacts:
        ball_a = balls[contact.index_a]
        ball_b = balls[contact.index_b]
        if resolve_ball_ball_contact(ball_a, ball_b, contact, table):
            applied += 1
    return applied
