"""Narrow-phase ball-cushion contact detection."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from sim_core.physics.ball import Ball
from sim_core.physics.table import CushionSegment, TableConfig
from sim_core.utils.vectors import dot

Vec2 = NDArray[np.float64]


@dataclass(frozen=True)
class BallRailContact:
    """Contact between a ball (index_a) and a cushion segment (index_b)."""

    index_a: int
    index_b: int
    closest_point: Vec2
    normal: Vec2
    penetration: float


def segment_projection_t(point: Vec2, segment: CushionSegment) -> float:
    """Return the clamped segment projection parameter for a query point."""
    edge = segment.end - segment.start
    edge_length_sq = dot(edge, edge)
    if edge_length_sq == 0.0:
        return 0.0

    t = dot(point - segment.start, edge) / edge_length_sq
    return float(np.clip(t, 0.0, 1.0))


def closest_point_on_segment(point: Vec2, segment: CushionSegment) -> Vec2:
    """Return the closest point on a finite segment to a query point."""
    t = segment_projection_t(point, segment)
    return segment.start + t * (segment.end - segment.start)


def find_ball_rail_contacts(
    balls: list[Ball],
    table: TableConfig,
    *,
    contact_tolerance: float = 1e-9,
) -> list[BallRailContact]:
    """
    Return ball-cushion contacts using closest-point penetration.

    Cushion normals point into the playable area, so penetration is measured as
    ball.radius minus signed distance from the cushion line into the table.
    """
    contacts: list[BallRailContact] = []

    for index_a, ball in enumerate(balls):
        if not ball.active:
            continue
        for index_b, cushion in enumerate(table.cushions):
            t = segment_projection_t(ball.position, cushion)
            if t <= contact_tolerance or t >= 1.0 - contact_tolerance:
                continue
            closest = closest_point_on_segment(ball.position, cushion)
            signed_distance = dot(ball.position - closest, cushion.normal)
            penetration = ball.radius - signed_distance

            if penetration >= -contact_tolerance:
                contacts.append(
                    BallRailContact(
                        index_a=index_a,
                        index_b=index_b,
                        closest_point=closest,
                        normal=cushion.normal,
                        penetration=max(0.0, penetration),
                    )
                )

    return contacts
