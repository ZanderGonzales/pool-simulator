import numpy as np
import pytest

from sim_core.physics.diagnostics import max_ball_overlap, total_kinetic_energy
from sim_core.physics.rack import RACK_BALL_COUNT, create_break_setup, triangle_rack, validate_no_overlaps
from sim_core.physics.simulation import Simulation, SimulationConfig
from sim_core.physics.table import TableConfig
from sim_core.utils.constants import BALL_RADIUS_M, ROW_SPACING_M
from sim_core.utils.vectors import norm, vec2


@pytest.fixture
def break_table() -> TableConfig:
    return TableConfig(
        rolling_resistance_coefficient=0.02,
        coefficient_of_restitution=0.95,
    )


@pytest.fixture
def break_simulation(break_table: TableConfig) -> Simulation:
    setup = create_break_setup(break_speed=3.0)
    return Simulation(
        balls=list(setup.balls),
        table=break_table,
        config=SimulationConfig(dt=0.005, collision_iterations=8, max_steps=50_000),
    )


def test_triangle_rack_has_15_non_overlapping_balls() -> None:
    rack = triangle_rack()
    assert len(rack.balls) == RACK_BALL_COUNT
    assert rack.apex_index == 0
    validate_no_overlaps(list(rack.balls))

    apex = rack.balls[rack.apex_index]
    row_one = rack.balls[1]
    assert apex.position[0] - row_one.position[0] == pytest.approx(ROW_SPACING_M, rel=1e-5)


def test_break_setup_places_cue_and_rack() -> None:
    setup = create_break_setup(break_speed=2.5)
    assert len(setup.balls) == RACK_BALL_COUNT + 1
    assert setup.cue_ball_index == RACK_BALL_COUNT
    assert setup.apex_index == 0

    cue = setup.balls[setup.cue_ball_index]
    apex = setup.balls[setup.apex_index]
    assert cue.position[0] > apex.position[0]
    assert cue.velocity[0] < 0.0
    min_distance = cue.radius + apex.radius
    assert norm(cue.position - apex.position) >= min_distance - 1e-9


def test_break_shot_produces_collisions(break_simulation: Simulation) -> None:
    initial_positions = [ball.position.copy() for ball in break_simulation.balls]
    for _ in range(10):
        break_simulation.step()

    object_ball_moved = any(
        not np.allclose(break_simulation.balls[index].position, initial_positions[index], atol=1e-9)
        for index in range(RACK_BALL_COUNT)
    )
    assert object_ball_moved


def test_break_shot_energy_does_not_increase(break_simulation: Simulation) -> None:
    previous_energy = total_kinetic_energy(break_simulation.balls)
    for _ in range(200):
        break_simulation.step()
        current_energy = total_kinetic_energy(break_simulation.balls)
        assert current_energy <= previous_energy + 1e-6
        previous_energy = current_energy


def test_break_shot_all_balls_eventually_stop(break_simulation: Simulation) -> None:
    steps = break_simulation.run(until_stopped=True)
    assert steps < break_simulation.config.max_steps
    assert break_simulation.all_stopped()


def test_no_persistent_overlaps_during_break(break_simulation: Simulation) -> None:
    for _ in range(300):
        break_simulation.step()
        assert max_ball_overlap(break_simulation.balls) < 1e-5
