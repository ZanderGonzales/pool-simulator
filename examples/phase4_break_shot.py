"""Phase 4 demo: 15-ball rack break shot."""

from sim_core.physics.diagnostics import count_moving_balls, total_kinetic_energy
from sim_core.physics.rack import create_break_setup
from sim_core.physics.simulation import Simulation, SimulationConfig
from sim_core.physics.table import TableConfig


def main() -> None:
    table = TableConfig(
        rolling_resistance_coefficient=0.02,
        coefficient_of_restitution=0.95,
    )
    setup = create_break_setup(break_speed=3.0)
    sim = Simulation(
        balls=list(setup.balls),
        table=table,
        config=SimulationConfig(dt=0.005, collision_iterations=8),
    )

    print("Phase 4 - break shot")
    print(
        f"Object balls: {len(setup.balls) - 1}, "
        f"cue speed: {setup.balls[setup.cue_ball_index].speed:.2f} m/s\n"
    )

    while not sim.all_stopped() and sim.time < 30.0:
        sim.step()

    moving = count_moving_balls(sim.balls, table.velocity_stop_threshold)
    print(f"t={sim.time:.3f}s  moving balls: {moving}")
    print(f"final kinetic energy: {total_kinetic_energy(sim.balls):.4f} J")
    print(f"all stopped: {sim.all_stopped()}")


if __name__ == "__main__":
    main()
