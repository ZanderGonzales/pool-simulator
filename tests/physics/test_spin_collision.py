import numpy as np
import pytest

from sim_core.physics.ball import Ball
from sim_core.physics.collision.detector import find_ball_ball_contacts
from sim_core.physics.collision.resolver import (
    resolve_ball_ball_contact,
    tangential_direction,
)
from sim_core.physics.table import TableConfig
from sim_core.utils.constants import BALL_RADIUS_M, DEFAULT_BALL_BALL_FRICTION
from sim_core.utils.vectors import dot, vec2


def _touching_with_spin(
    *,
    omega_a: float = 5.0,
    omega_b: float = 0.0,
    speed_a: float = 1.0,
    speed_b: float = -0.5,
    ball_ball_friction: float = DEFAULT_BALL_BALL_FRICTION,
) -> tuple[Ball, Ball, TableConfig]:
    r = BALL_RADIUS_M
    table = TableConfig(
        rolling_resistance_coefficient=0.0,
        coefficient_of_restitution=0.95,
        ball_ball_friction=ball_ball_friction,
    )
    overlap = 1e-6
    ball_a = Ball(
        id=0,
        position=vec2(-r + overlap, 0.0),
        velocity=vec2(speed_a, 0.0),
        omega=omega_a,
    )
    ball_b = Ball(
        id=1,
        position=vec2(r - overlap, 0.0),
        velocity=vec2(speed_b, 0.0),
        omega=omega_b,
    )
    return ball_a, ball_b, table


def test_tangential_impulse_changes_omega() -> None:
    ball_a, ball_b, table = _touching_with_spin()
    omega_a_before = ball_a.omega
    omega_b_before = ball_b.omega
    contacts = find_ball_ball_contacts([ball_a, ball_b])
    assert len(contacts) == 1

    resolve_ball_ball_contact(ball_a, ball_b, contacts[0], table)

    assert ball_a.omega != pytest.approx(omega_a_before)
    assert ball_b.omega != pytest.approx(omega_b_before)


def test_friction_impulse_capped() -> None:
    ball_a, ball_b, table = _touching_with_spin(
        omega_a=20.0,
        omega_b=-15.0,
        speed_a=2.0,
        speed_b=-1.0,
    )
    mu_bb = table.ball_ball_friction
    contacts = find_ball_ball_contacts([ball_a, ball_b])
    normal = contacts[0].normal
    tangent = tangential_direction(normal)

    v_rel_n = dot(ball_b.velocity - ball_a.velocity, normal)
    inv_mass_sum = (1.0 / ball_a.mass) + (1.0 / ball_b.mass)
    normal_impulse = -(1.0 + table.coefficient_of_restitution) * v_rel_n / inv_mass_sum

    v_a = ball_a.velocity.copy()
    v_b = ball_b.velocity.copy()

    resolve_ball_ball_contact(ball_a, ball_b, contacts[0], table)

    delta_v_a = ball_a.velocity - v_a
    delta_v_b = ball_b.velocity - v_b
    tangential_impulse_a = dot(delta_v_a, tangent) * ball_a.mass
    tangential_impulse_b = dot(delta_v_b, tangent) * ball_b.mass

    assert abs(tangential_impulse_a) == pytest.approx(abs(tangential_impulse_b), rel=1e-4)
    assert abs(tangential_impulse_a) <= mu_bb * normal_impulse + 1e-9
    assert normal_impulse > 0.0
