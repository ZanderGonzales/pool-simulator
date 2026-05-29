import numpy as np
import pytest

from sim_core.physics.ball import Ball
from sim_core.physics.integrator import (
    apply_rolling_resistance,
    rolling_deceleration,
    step_ball,
    stopping_distance,
    stopping_time,
)
from sim_core.physics.table import TableConfig
from sim_core.utils.vectors import vec2


def test_position_update_no_friction() -> None:
    table = TableConfig(
        rolling_resistance_coefficient=0.0,
        sliding_friction_coefficient=0.0,
    )
    ball = Ball(id=0, position=vec2(0.0, 0.0), velocity=vec2(2.0, 1.0))
    dt = 0.01
    n_steps = 100

    for _ in range(n_steps):
        step_ball(ball, table, dt)

    t = n_steps * dt
    expected = vec2(2.0, 1.0) * t
    np.testing.assert_allclose(ball.position, expected, rtol=1e-5)
    np.testing.assert_allclose(ball.velocity, vec2(2.0, 1.0), rtol=1e-5)


def test_velocity_loses_constant_speed_under_rolling_resistance() -> None:
    mu = 0.02
    dt = 0.01
    n_steps = 50
    table = TableConfig(rolling_resistance_coefficient=mu)
    ball = Ball(id=0, position=vec2(0.0, 0.0), velocity=vec2(1.0, 0.0))

    for _ in range(n_steps):
        apply_rolling_resistance(ball, table, dt)

    expected_speed = 1.0 - rolling_deceleration(table) * dt * n_steps
    assert ball.speed == pytest.approx(expected_speed, rel=1e-5)


def test_stopping_clamp() -> None:
    table = TableConfig(velocity_stop_threshold=0.01)
    ball = Ball(id=0, position=vec2(0.0, 0.0), velocity=vec2(0.005, 0.0))

    apply_rolling_resistance(ball, table, 0.01)

    np.testing.assert_allclose(ball.velocity, vec2(0.0, 0.0))


def test_stopping_time_and_distance_match_kinematics() -> None:
    table = TableConfig(rolling_resistance_coefficient=0.02)
    v0 = 1.0
    a = rolling_deceleration(table)

    assert stopping_time(v0, table) == pytest.approx(v0 / a)
    assert stopping_distance(v0, table) == pytest.approx(v0 * v0 / (2.0 * a))


def test_single_step_stops_at_exact_stopping_distance() -> None:
    table = TableConfig(
        rolling_resistance_coefficient=0.02,
        sliding_friction_coefficient=0.0,
    )
    v0 = 1.0
    ball = Ball(id=0, position=vec2(0.0, 0.0), velocity=vec2(v0, 0.0))

    step_ball(ball, table, stopping_time(v0, table) + 1.0)

    np.testing.assert_allclose(ball.position, vec2(stopping_distance(v0, table), 0.0))
    np.testing.assert_allclose(ball.velocity, vec2(0.0, 0.0))


def test_single_step_matches_closed_form_before_stopping() -> None:
    table = TableConfig(
        rolling_resistance_coefficient=0.02,
        sliding_friction_coefficient=0.0,
    )
    v0 = 1.0
    dt = 0.5
    ball = Ball(id=0, position=vec2(0.0, 0.0), velocity=vec2(v0, 0.0))
    a = rolling_deceleration(table)

    step_ball(ball, table, dt)

    np.testing.assert_allclose(ball.position, vec2(v0 * dt - 0.5 * a * dt * dt, 0.0))
    assert ball.speed == pytest.approx(v0 - a * dt)
