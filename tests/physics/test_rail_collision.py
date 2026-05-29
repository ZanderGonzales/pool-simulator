import numpy as np
import pytest

from sim_core.physics.ball import Ball
from sim_core.physics.collision.rail_detector import (
    closest_point_on_segment,
    find_ball_rail_contacts,
)
from sim_core.physics.collision.rail_resolver import resolve_ball_rail_contact
from sim_core.physics.simulation import Simulation, SimulationConfig
from sim_core.physics.table import TableConfig
from sim_core.utils.constants import BALL_RADIUS_M
from sim_core.utils.vectors import vec2


def _kinetic_energy(ball: Ball) -> float:
    return 0.5 * ball.mass * ball.speed**2


def test_detects_ball_rail_penetration_by_closest_point() -> None:
    table = TableConfig(rolling_resistance_coefficient=0.0, rail_friction=0.0)
    right_cushion = table.cushions[1]
    ball = Ball(
        id=0,
        position=vec2(table.width - 0.5 * BALL_RADIUS_M, 0.5 * table.height),
        velocity=vec2(0.0, 0.0),
    )

    closest = closest_point_on_segment(ball.position, right_cushion)
    contacts = find_ball_rail_contacts([ball], table)

    np.testing.assert_allclose(closest, vec2(table.width, 0.5 * table.height))
    assert len(contacts) == 1
    assert contacts[0].penetration == pytest.approx(0.5 * ball.radius)


def test_separates_overlap_when_moving_away_from_cushion() -> None:
    """Penetration is corrected even when dot(v, n) >= 0 (no velocity reflection)."""
    table = TableConfig(rolling_resistance_coefficient=0.0, rail_friction=0.0)
    ball = Ball(
        id=0,
        position=vec2(table.width - 0.5 * BALL_RADIUS_M, 0.5 * table.height),
        velocity=vec2(-0.5, 0.0),
    )
    velocity_before = ball.velocity.copy()
    contacts = find_ball_rail_contacts([ball], table)
    assert len(contacts) == 1

    applied = resolve_ball_rail_contact(ball, contacts[0], table)

    assert applied is False
    np.testing.assert_allclose(ball.velocity, velocity_before)
    assert ball.position[0] == pytest.approx(table.width - ball.radius)
    assert ball.position[0] < table.width - 0.5 * ball.radius


def test_specular_reflection_angle_off_cushion() -> None:
    table = TableConfig(
        rolling_resistance_coefficient=0.0,
        sliding_friction_coefficient=0.0,
        coefficient_of_restitution=1.0,
        rail_friction=0.0,
    )
    ball = Ball(
        id=0,
        position=vec2(table.width - 0.5 * BALL_RADIUS_M, 0.5 * table.height),
        velocity=vec2(1.0, 0.5),
    )
    speed_before = ball.speed
    sim = Simulation(
        balls=[ball],
        table=table,
        config=SimulationConfig(dt=0.001),
    )

    sim.step()

    np.testing.assert_allclose(ball.velocity, vec2(-1.0, 0.5), rtol=1e-12, atol=1e-12)
    assert ball.speed == pytest.approx(speed_before)


def test_high_speed_ball_does_not_tunnel_through_rail() -> None:
    table = TableConfig(
        rolling_resistance_coefficient=0.0,
        coefficient_of_restitution=1.0,
        rail_friction=0.0,
    )
    ball = Ball(
        id=0,
        position=vec2(0.5 * table.width, 0.5 * table.height),
        velocity=vec2(500.0, 0.0),
    )
    sim = Simulation(
        balls=[ball],
        table=table,
        config=SimulationConfig(dt=0.05),
    )

    sim.step()

    assert ball.position[0] <= table.width - ball.radius + 1e-12
    assert ball.velocity[0] < 0.0


def test_inelastic_cushion_reduces_energy_by_restitution() -> None:
    e = 0.5
    table = TableConfig(
        rolling_resistance_coefficient=0.0,
        sliding_friction_coefficient=0.0,
        coefficient_of_restitution=e,
        rail_friction=0.0,
    )
    ball = Ball(
        id=0,
        position=vec2(table.width - 0.5 * BALL_RADIUS_M, 0.5 * table.height),
        velocity=vec2(2.0, 0.0),
    )
    ke_before = _kinetic_energy(ball)
    sim = Simulation(
        balls=[ball],
        table=table,
        config=SimulationConfig(dt=0.001),
    )

    sim.step()

    ke_after = _kinetic_energy(ball)
    assert ke_after == pytest.approx(ke_before * e * e)
    assert ke_after < ke_before
