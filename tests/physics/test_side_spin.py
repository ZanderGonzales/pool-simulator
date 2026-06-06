"""Phase 7: 3D angular velocity and side-spin behavior."""

import numpy as np

from sim_core.physics.ball import Ball
from sim_core.physics.shot import CueStrike
from sim_core.physics.simulation import Simulation, SimulationConfig
from sim_core.physics.table import TableConfig
from sim_core.utils.vectors import norm, vec2, vec3


def test_cue_strike_side_offset_spins_about_aim() -> None:
    strike = CueStrike(
        speed=2.0,
        aim_direction=vec2(1.0, 0.0),
        hit_offset_perpendicular=0.5,
    )
    omega = strike.to_angular_velocity()
    np.testing.assert_allclose(omega[0], 2.0 / 0.028575 * 0.5, rtol=1e-5)
    assert abs(omega[1]) < 1e-9
    assert omega[2] == 0.0


def test_side_spin_curves_path_with_swerve() -> None:
    table = TableConfig(
        rolling_resistance_coefficient=0.0,
        sliding_friction_coefficient=0.0,
        spin_decay_rate=0.0,
        swerve_coefficient=0.15,
    )
    ball = Ball(
        id=0,
        position=vec2(0.2, 0.5),
        velocity=vec2(2.0, 0.0),
        angular_velocity=vec3(0.0, 12.0, 0.0),
    )
    sim = Simulation(balls=[ball], table=table, config=SimulationConfig(dt=0.01))

    for _ in range(100):
        sim.step()

    assert abs(ball.velocity[1]) > 0.05


def test_ball_ball_transfers_spin() -> None:
    table = TableConfig(
        rolling_resistance_coefficient=0.0,
        sliding_friction_coefficient=0.0,
        ball_ball_friction=0.2,
    )
    ball_a = Ball(
        id=0,
        position=vec2(0.0, 0.0),
        velocity=vec2(2.0, 0.0),
        omega=12.0,
    )
    ball_b = Ball(
        id=1,
        position=vec2(2.05 * ball_a.radius, 0.0),
        velocity=vec2(0.0, 0.0),
        omega=0.0,
    )
    sim = Simulation(
        balls=[ball_a, ball_b],
        table=table,
        config=SimulationConfig(dt=0.001, collision_iterations=4),
    )
    sim.step()

    assert ball_b.spin_speed > 0.01
