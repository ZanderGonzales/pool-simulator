import pytest

from sim_core.physics.ball import Ball
from sim_core.physics.simulation import Simulation, SimulationConfig
from sim_core.physics.table import TableConfig
from sim_core.utils.vectors import vec2


@pytest.fixture
def table() -> TableConfig:
    return TableConfig(
        rolling_resistance_coefficient=0.02,
        velocity_stop_threshold=1e-4,
    )


@pytest.fixture
def single_ball() -> Ball:
    return Ball(id=0, position=vec2(0.0, 0.0), velocity=vec2(1.0, 0.0))


@pytest.fixture
def simulation(table: TableConfig, single_ball: Ball) -> Simulation:
    return Simulation(
        balls=[single_ball],
        table=table,
        config=SimulationConfig(dt=0.01),
    )
