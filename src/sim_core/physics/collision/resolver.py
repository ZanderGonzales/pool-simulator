"""Impulse-based ball-ball collision resolution with normal and tangential impulses."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from sim_core.physics.ball import Ball
from sim_core.physics.collision.detector import BallBallContact
from sim_core.physics.spin_integrator import sphere_inertia
from sim_core.physics.table import TableConfig
from sim_core.utils.vectors import cross3, dot, norm3, vec2, vec2_to_vec3, vec3

Vec2 = NDArray[np.float64]
Vec3 = NDArray[np.float64]


def separate_balls(ball_a: Ball, ball_b: Ball, normal: Vec2, penetration: float) -> None:
    """Push overlapping balls apart along the contact normal (equal split)."""
    if penetration <= 0.0:
        return
    half = 0.5 * penetration
    ball_a.position = ball_a.position - normal * half
    ball_b.position = ball_b.position + normal * half


def tangential_direction(normal: Vec2) -> Vec2:
    """Unit tangent CCW-perpendicular to the contact normal."""
    return vec2(-normal[1], normal[0])


def _contact_offset(normal: Vec2, radius: float, *, toward_other: bool) -> Vec3:
    sign = 1.0 if toward_other else -1.0
    return vec3(sign * normal[0] * radius, sign * normal[1] * radius, 0.0)


def _surface_velocity_xy(ball: Ball, offset: Vec3) -> Vec2:
    v = vec2_to_vec3(ball.velocity)
    return (v + cross3(ball.angular_velocity, offset))[:2]


def relative_tangential_velocity(
    ball_a: Ball,
    ball_b: Ball,
    tangent: Vec2,
    normal: Vec2,
) -> float:
    """Relative tangential velocity at the contact including 3D spin."""
    r = ball_a.radius
    off_a = _contact_offset(normal, r, toward_other=True)
    off_b = _contact_offset(normal, r, toward_other=False)
    v_a = _surface_velocity_xy(ball_a, off_a)
    v_b = _surface_velocity_xy(ball_b, off_b)
    return dot(v_b - v_a, tangent)


def resolve_ball_ball_contact(
    ball_a: Ball,
    ball_b: Ball,
    contact: BallBallContact,
    table: TableConfig,
) -> bool:
    """
    Apply normal and capped tangential impulses for equal-mass spheres.

    Normal n points from ball_a to ball_b. Tangential impulse is limited by
    Coulomb friction |J_t| <= mu_bb * |J_n|. Returns True if a normal impulse
    was applied.
    """
    normal = contact.normal
    tangent = tangential_direction(normal)
    applied_impulse = False
    normal_impulse = 0.0

    v_rel_n = dot(ball_b.velocity - ball_a.velocity, normal)
    if v_rel_n < 0.0:
        e = table.coefficient_of_restitution
        inv_mass_sum = (1.0 / ball_a.mass) + (1.0 / ball_b.mass)
        normal_impulse = -(1.0 + e) * v_rel_n / inv_mass_sum

        ball_a.velocity = ball_a.velocity - (normal_impulse / ball_a.mass) * normal
        ball_b.velocity = ball_b.velocity + (normal_impulse / ball_b.mass) * normal
        applied_impulse = True

    if normal_impulse > 0.0 and table.ball_ball_friction > 0.0:
        _apply_tangential_impulse(
            ball_a,
            ball_b,
            tangent,
            normal,
            table,
            normal_impulse,
        )

    if contact.penetration > 0.0:
        separate_balls(ball_a, ball_b, normal, contact.penetration)

    return applied_impulse


def _apply_tangential_impulse(
    ball_a: Ball,
    ball_b: Ball,
    tangent: Vec2,
    normal: Vec2,
    table: TableConfig,
    normal_impulse: float,
) -> float:
    """
    Apply tangential impulse along tangent and update spin on both balls.

    Returns the tangential impulse magnitude applied (signed).
    """
    r = ball_a.radius
    inertia = sphere_inertia(ball_a.mass, r)
    off_a = _contact_offset(normal, r, toward_other=True)
    off_b = _contact_offset(normal, r, toward_other=False)
    tangent_3d = vec3(tangent[0], tangent[1], 0.0)
    lever_a = norm3(cross3(off_a, tangent_3d))
    lever_b = norm3(cross3(off_b, tangent_3d))
    inv_mass_sum = (1.0 / ball_a.mass) + (1.0 / ball_b.mass)
    inv_inertia_sum = (lever_a * lever_a + lever_b * lever_b) / inertia
    inv_tangential = inv_mass_sum + inv_inertia_sum

    v_rel_t = relative_tangential_velocity(ball_a, ball_b, tangent, normal)
    if abs(v_rel_t) < 1e-12:
        return 0.0

    tangential_impulse = -v_rel_t / inv_tangential
    max_tangential = table.ball_ball_friction * normal_impulse
    tangential_impulse = float(np.clip(tangential_impulse, -max_tangential, max_tangential))

    impulse_vec = vec3(tangent[0] * tangential_impulse, tangent[1] * tangential_impulse, 0.0)
    ball_a.velocity = ball_a.velocity - (tangential_impulse / ball_a.mass) * tangent
    ball_b.velocity = ball_b.velocity + (tangential_impulse / ball_b.mass) * tangent
    ball_a.angular_velocity = ball_a.angular_velocity - cross3(off_a, impulse_vec) / inertia
    ball_b.angular_velocity = ball_b.angular_velocity + cross3(off_b, impulse_vec) / inertia

    return tangential_impulse


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
