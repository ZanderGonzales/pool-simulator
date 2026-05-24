"""Narrow-phase ball-ball contact detection."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from sim_core.physics.ball import Ball
from sim_core.utils.vectors import normalize, norm, vec2

Vec2 = NDArray[np.float64]


@dataclass(frozen=True)
class BallBallContact:
    """Contact between two balls: normal points from ball_a toward ball_b."""

    index_a: int
    index_b: int
    normal: Vec2
    penetration: float


def find_ball_ball_contacts(
    balls: list[Ball],
    *,
    contact_tolerance: float = 1e-9,
) -> list[BallBallContact]:
    """
    Return contacts where center distance < r_a + r_b.

    The contact normal is the unit vector from ball_a center to ball_b center.
    """
    contacts: list[BallBallContact] = []

    for i in range(len(balls)):
        ball_a = balls[i]
        if not ball_a.active:
            continue
        for j in range(i + 1, len(balls)):
            ball_b = balls[j]
            if not ball_b.active:
                continue

            delta = ball_b.position - ball_a.position
            distance = norm(delta)
            min_distance = ball_a.radius + ball_b.radius

            if distance < contact_tolerance:
                normal = vec2(1.0, 0.0)
                penetration = min_distance
            else:
                normal = normalize(delta)
                penetration = max(0.0, min_distance - distance)

            if distance <= min_distance + contact_tolerance:
                contacts.append(
                    BallBallContact(
                        index_a=i,
                        index_b=j,
                        normal=normal,
                        penetration=penetration,
                    )
                )

    return contacts
