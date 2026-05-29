import pytest

from sim_core.physics.ball import Ball
from sim_core.physics.collision.rail_detector import find_ball_rail_contacts
from sim_core.physics.collision.rail_resolver import resolve_ball_rail_contact
from sim_core.physics.table import TableConfig
from sim_core.utils.constants import BALL_RADIUS_M
from sim_core.utils.vectors import vec2


def test_glancing_rail_contact_changes_omega() -> None:
    table = TableConfig(rolling_resistance_coefficient=0.0, coefficient_of_restitution=1.0)
    ball = Ball(
        id=0,
        position=vec2(table.width - 0.5 * BALL_RADIUS_M, 0.5 * table.height),
        velocity=vec2(1.0, 0.8),
        omega=0.0,
    )
    contacts = find_ball_rail_contacts([ball], table)
    assert len(contacts) == 1

    resolve_ball_rail_contact(ball, contacts[0], table)

    assert abs(ball.omega) > 0.0


def test_head_on_rail_contact_keeps_zero_omega() -> None:
    table = TableConfig(rolling_resistance_coefficient=0.0, coefficient_of_restitution=1.0)
    ball = Ball(
        id=0,
        position=vec2(table.width - 0.5 * BALL_RADIUS_M, 0.5 * table.height),
        velocity=vec2(1.0, 0.0),
        omega=0.0,
    )
    contacts = find_ball_rail_contacts([ball], table)
    assert len(contacts) == 1

    resolve_ball_rail_contact(ball, contacts[0], table)

    assert ball.omega == pytest.approx(0.0, abs=1e-12)
