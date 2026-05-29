import numpy as np

from sim_core.physics.ball import Ball
from sim_core.physics.simulation import Simulation, SimulationConfig
from sim_core.physics.table import TableConfig
from sim_core.utils.vectors import vec2


def test_corner_pocket_captures_ball() -> None:
    table = TableConfig(rolling_resistance_coefficient=0.0, coefficient_of_restitution=1.0)
    pocket = table.pockets[0]
    ball = Ball(id=0, position=pocket.center + vec2(0.25 * pocket.radius, 0.0), velocity=vec2(0.0, 0.0))
    sim = Simulation(balls=[ball], table=table, config=SimulationConfig(dt=0.01))

    sim.step()

    assert ball.active is False
    np.testing.assert_allclose(ball.velocity, vec2(0.0, 0.0))


def test_side_pocket_captures_ball() -> None:
    table = TableConfig(rolling_resistance_coefficient=0.0, coefficient_of_restitution=1.0)
    side_pocket = table.pockets[4]
    ball = Ball(id=1, position=side_pocket.center + vec2(0.0, 0.5 * side_pocket.radius), velocity=vec2(0.0, -0.1))
    sim = Simulation(balls=[ball], table=table, config=SimulationConfig(dt=0.01))

    sim.step()

    assert ball.active is False


def test_ball_near_rail_not_pocketed() -> None:
    table = TableConfig(rolling_resistance_coefficient=0.0, coefficient_of_restitution=1.0)
    ball = Ball(
        id=2,
        position=vec2(table.width * 0.5, table.height * 0.5),
        velocity=vec2(0.0, 0.0),
    )
    sim = Simulation(balls=[ball], table=table, config=SimulationConfig(dt=0.01))

    sim.step()

    assert ball.active is True


def test_cue_ball_scratch_deactivates_cue() -> None:
    table = TableConfig(rolling_resistance_coefficient=0.0, coefficient_of_restitution=1.0)
    scratch_target = table.pockets[1]
    cue_ball = Ball(id=15, position=scratch_target.center + vec2(-0.2 * scratch_target.radius, 0.0), velocity=vec2(0.0, 0.0))
    object_ball = Ball(id=1, position=vec2(table.width * 0.5, table.height * 0.5), velocity=vec2(0.0, 0.0))
    sim = Simulation(balls=[object_ball, cue_ball], table=table, config=SimulationConfig(dt=0.01))

    sim.step()

    assert cue_ball.active is False
    assert object_ball.active is True
