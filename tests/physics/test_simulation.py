import numpy as np
import pytest

from sim_core.physics.ball import Ball
from sim_core.physics.simulation import Simulation, SimulationConfig
from sim_core.physics.table import TableConfig
from sim_core.utils.vectors import vec2


def _interior_start():
    """Position away from default cushions (table origin is a corner)."""
    return vec2(1.0, 0.5)


def test_simulation_step_advances_time() -> None:
    sim = Simulation(
        balls=[Ball(id=0, position=_interior_start(), velocity=vec2(1, 0))],
        table=TableConfig(rolling_resistance_coefficient=0.0),
        config=SimulationConfig(dt=0.01),
    )
    sim.step()
    assert sim.time == pytest.approx(0.01)


def test_all_balls_stop() -> None:
    sim = Simulation(
        balls=[Ball(id=0, position=_interior_start(), velocity=vec2(1, 0))],
        table=TableConfig(rolling_resistance_coefficient=0.02, velocity_stop_threshold=1e-3),
        config=SimulationConfig(dt=0.01, max_steps=10_000),
    )
    steps = sim.run(until_stopped=True)

    assert steps < 10_000
    assert sim.all_stopped()
    assert sim.balls[0].speed == 0.0


def test_run_fixed_steps() -> None:
    start = _interior_start()
    sim = Simulation(
        balls=[Ball(id=0, position=start, velocity=vec2(1, 0))],
        table=TableConfig(rolling_resistance_coefficient=0.0),
        config=SimulationConfig(dt=0.01),
    )
    executed = sim.run(steps=10)
    assert executed == 10
    np.testing.assert_allclose(sim.balls[0].position, start + vec2(0.1, 0.0), rtol=1e-5)


def test_snapshot_structure() -> None:
    sim = Simulation(
        balls=[Ball(id=0, position=vec2(1, 2), velocity=vec2(0.5, 0))],
        table=TableConfig(),
        config=SimulationConfig(dt=0.01),
    )
    snap = sim.snapshot()
    assert snap["time"] == 0.0
    assert len(snap["balls"]) == 1
    assert snap["balls"][0]["id"] == 0
    np.testing.assert_allclose(snap["balls"][0]["position"], vec2(1, 2))
