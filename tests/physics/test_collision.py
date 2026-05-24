import numpy as np
import pytest

from sim_core.physics.ball import Ball
from sim_core.physics.collision.detector import find_ball_ball_contacts
from sim_core.physics.collision.resolver import resolve_ball_ball_contact
from sim_core.physics.simulation import Simulation, SimulationConfig
from sim_core.physics.table import TableConfig
from sim_core.utils.constants import BALL_RADIUS_M
from sim_core.utils.vectors import vec2


def _touching_pair(
    speed_a: float = 1.0,
    speed_b: float = -1.0,
) -> tuple[Ball, Ball, TableConfig]:
    r = BALL_RADIUS_M
    table = TableConfig(
        rolling_resistance_coefficient=0.0,
        coefficient_of_restitution=1.0,
    )
    overlap = 1e-6
    ball_a = Ball(id=0, position=vec2(-r + overlap, 0.0), velocity=vec2(speed_a, 0.0))
    ball_b = Ball(id=1, position=vec2(r - overlap, 0.0), velocity=vec2(speed_b, 0.0))
    return ball_a, ball_b, table


def test_detects_overlapping_balls() -> None:
    ball_a = Ball(id=0, position=vec2(0.0, 0.0), velocity=vec2(0.0, 0.0))
    ball_b = Ball(
        id=1,
        position=vec2(2.0 * ball_a.radius - 0.001, 0.0),
        velocity=vec2(0.0, 0.0),
    )

    contacts = find_ball_ball_contacts([ball_a, ball_b])
    assert len(contacts) == 1
    assert contacts[0].penetration > 0.0


def test_head_on_elastic_swaps_velocities() -> None:
    ball_a, ball_b, table = _touching_pair()
    contacts = find_ball_ball_contacts([ball_a, ball_b])
    assert len(contacts) == 1

    resolve_ball_ball_contact(ball_a, ball_b, contacts[0], table)

    np.testing.assert_allclose(ball_a.velocity, vec2(-1.0, 0.0), rtol=1e-5)
    np.testing.assert_allclose(ball_b.velocity, vec2(1.0, 0.0), rtol=1e-5)


def test_momentum_conservation() -> None:
    ball_a, ball_b, table = _touching_pair(speed_a=2.0, speed_b=-0.5)
    contacts = find_ball_ball_contacts([ball_a, ball_b])

    p_before = ball_a.mass * ball_a.velocity + ball_b.mass * ball_b.velocity
    resolve_ball_ball_contact(ball_a, ball_b, contacts[0], table)
    p_after = ball_a.mass * ball_a.velocity + ball_b.mass * ball_b.velocity

    np.testing.assert_allclose(p_before, p_after, rtol=1e-5)


def test_kinetic_energy_near_conserved_for_elastic() -> None:
    ball_a, ball_b, table = _touching_pair()
    contacts = find_ball_ball_contacts([ball_a, ball_b])

    ke_before = 0.5 * ball_a.mass * ball_a.speed**2 + 0.5 * ball_b.mass * ball_b.speed**2
    resolve_ball_ball_contact(ball_a, ball_b, contacts[0], table)
    ke_after = 0.5 * ball_a.mass * ball_a.speed**2 + 0.5 * ball_b.mass * ball_b.speed**2

    assert ke_after == pytest.approx(ke_before, rel=1e-5)


def test_inelastic_collision_reduces_energy() -> None:
    ball_a, ball_b, table = _touching_pair()
    table = TableConfig(
        rolling_resistance_coefficient=0.0,
        coefficient_of_restitution=0.0,
    )
    contacts = find_ball_ball_contacts([ball_a, ball_b])

    ke_before = 0.5 * ball_a.mass * ball_a.speed**2 + 0.5 * ball_b.mass * ball_b.speed**2
    resolve_ball_ball_contact(ball_a, ball_b, contacts[0], table)
    ke_after = 0.5 * ball_a.mass * ball_a.speed**2 + 0.5 * ball_b.mass * ball_b.speed**2

    assert ke_after < ke_before
    np.testing.assert_allclose(ball_a.velocity, vec2(0.0, 0.0), atol=1e-5)
    np.testing.assert_allclose(ball_b.velocity, vec2(0.0, 0.0), atol=1e-5)


def test_separation_removes_overlap() -> None:
    r = BALL_RADIUS_M
    ball_a = Ball(id=0, position=vec2(0.0, 0.0), velocity=vec2(0.0, 0.0))
    ball_b = Ball(id=1, position=vec2(1.5 * r, 0.0), velocity=vec2(0.0, 0.0))
    table = TableConfig(coefficient_of_restitution=0.95)

    contacts = find_ball_ball_contacts([ball_a, ball_b])
    resolve_ball_ball_contact(ball_a, ball_b, contacts[0], table)

    distance = np.linalg.norm(ball_b.position - ball_a.position)
    assert distance >= 2.0 * r - 1e-9


def test_non_head_on_momentum_and_energy_conserved() -> None:
    """Frictionless elastic contact conserves momentum and kinetic energy."""
    r = BALL_RADIUS_M
    table = TableConfig(rolling_resistance_coefficient=0.0, coefficient_of_restitution=1.0)
    ball_a = Ball(id=0, position=vec2(0.0, 0.0), velocity=vec2(2.0, 0.5))
    ball_b = Ball(
        id=1,
        position=vec2(1.8 * r, 0.3 * r),
        velocity=vec2(-1.0, -0.5),
    )

    contacts = find_ball_ball_contacts([ball_a, ball_b])
    assert len(contacts) == 1

    p_before = ball_a.mass * ball_a.velocity + ball_b.mass * ball_b.velocity
    ke_before = 0.5 * ball_a.mass * ball_a.speed**2 + 0.5 * ball_b.mass * ball_b.speed**2

    resolve_ball_ball_contact(ball_a, ball_b, contacts[0], table)

    p_after = ball_a.mass * ball_a.velocity + ball_b.mass * ball_b.velocity
    ke_after = 0.5 * ball_a.mass * ball_a.speed**2 + 0.5 * ball_b.mass * ball_b.speed**2

    np.testing.assert_allclose(p_before, p_after, rtol=1e-5)
    assert ke_after == pytest.approx(ke_before, rel=1e-5)


def test_simulation_resolves_collision_step() -> None:
    r = BALL_RADIUS_M
    ball_a = Ball(id=0, position=vec2(-r, 0.0), velocity=vec2(2.0, 0.0))
    ball_b = Ball(id=1, position=vec2(r, 0.0), velocity=vec2(-2.0, 0.0))
    sim = Simulation(
        balls=[ball_a, ball_b],
        table=TableConfig(
            rolling_resistance_coefficient=0.0,
            coefficient_of_restitution=1.0,
        ),
        config=SimulationConfig(dt=0.001, collision_iterations=4),
    )

    sim.step()

    assert ball_a.velocity[0] < 0.0
    assert ball_b.velocity[0] > 0.0
