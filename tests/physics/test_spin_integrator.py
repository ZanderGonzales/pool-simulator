import numpy as np
import pytest

from sim_core.physics.ball import Ball
from sim_core.physics.integrator import step_ball
from sim_core.physics.spin_integrator import (
    apply_cloth_friction,
    is_pure_rolling,
    slip_velocity,
)
from sim_core.physics.table import TableConfig
from sim_core.utils.constants import BALL_RADIUS_M
from sim_core.utils.vectors import norm, vec2


def test_slip_velocity_matches_motion_aligned_model() -> None:
    ball = Ball(id=0, position=vec2(0.0, 0.0), velocity=vec2(1.0, 0.0), omega=0.5 / BALL_RADIUS_M)
    expected = ball.velocity - ball.omega * ball.radius * (ball.velocity / ball.speed)
    np.testing.assert_allclose(slip_velocity(ball), expected, rtol=1e-9)


def test_slip_velocity_stationary() -> None:
    ball = Ball(id=0, position=vec2(0.0, 0.0), velocity=vec2(0.0, 0.0), omega=0.0)
    np.testing.assert_allclose(slip_velocity(ball), vec2(0.0, 0.0), atol=1e-9)


def test_pure_rolling_when_speed_matches_rim() -> None:
    speed = 1.0
    ball = Ball(
        id=0,
        position=vec2(0.0, 0.0),
        velocity=vec2(speed, 0.0),
        omega=speed / BALL_RADIUS_M,
    )
    assert is_pure_rolling(ball, tolerance=1e-6)


def test_spin_decays_under_table_friction(spin_table: TableConfig) -> None:
    ball = Ball(
        id=0,
        position=vec2(0.0, 0.0),
        velocity=vec2(0.05, 0.0),
        omega=5.0,
    )
    dt = 0.05
    for _ in range(80):
        apply_cloth_friction(ball, spin_table, dt)

    assert abs(ball.omega) < abs(5.0)
    assert abs(ball.omega) < spin_table.omega_stop_threshold or ball.speed < spin_table.velocity_stop_threshold


def test_sliding_ball_sheds_spin_and_speed(spin_table: TableConfig) -> None:
    ball = Ball(
        id=0,
        position=vec2(0.0, 0.0),
        velocity=vec2(2.0, 0.0),
        omega=30.0,
    )
    initial_speed = ball.speed
    initial_omega = abs(ball.omega)
    dt = 0.01

    for _ in range(200):
        apply_cloth_friction(ball, spin_table, dt)

    assert ball.speed < initial_speed
    assert abs(ball.omega) < initial_omega


def test_skidding_without_spin_uses_sliding_path() -> None:
    table = TableConfig(
        rolling_resistance_coefficient=0.0,
        sliding_friction_coefficient=0.2,
        spin_decay_rate=0.0,
    )
    ball = Ball(id=0, position=vec2(0.0, 0.0), velocity=vec2(2.0, 0.0), omega=0.0)
    initial_speed = ball.speed

    step_ball(ball, table, 0.05)

    assert initial_speed > ball.speed
    assert norm(slip_velocity(ball)) < initial_speed


def test_cloth_friction_does_not_increase_energy(spin_table: TableConfig) -> None:
    ball = Ball(
        id=0,
        position=vec2(0.0, 0.0),
        velocity=vec2(1.5, 0.2),
        omega=8.0,
    )
    inertia = 0.4 * ball.mass * ball.radius * ball.radius
    initial = 0.5 * ball.mass * ball.speed**2 + 0.5 * inertia * ball.omega**2
    dt = 0.01

    for _ in range(100):
        apply_cloth_friction(ball, spin_table, dt)
        energy = 0.5 * ball.mass * ball.speed**2 + 0.5 * inertia * ball.omega**2
        assert energy <= initial + 1e-6
