"""Phase 3 demo: ball bounces off the right cushion with no rolling resistance."""

from sim_core.physics.ball import Ball
from sim_core.physics.simulation import Simulation, SimulationConfig
from sim_core.physics.table import TableConfig
from sim_core.utils.constants import BALL_RADIUS_M
from sim_core.utils.vectors import vec2


def main() -> None:
    table = TableConfig(
        rolling_resistance_coefficient=0.0,
        coefficient_of_restitution=0.95,
    )
    r = BALL_RADIUS_M
    ball = Ball(
        id=0,
        position=vec2(0.5 * table.width, 0.5 * table.height),
        velocity=vec2(2.0, 0.3),
    )
    sim = Simulation(
        balls=[ball],
        table=table,
        config=SimulationConfig(dt=0.01, collision_iterations=4),
    )

    print("Phase 3 — rail bounce")
    print(f"Table: {table.width:.2f} m x {table.height:.2f} m")
    print(f"Restitution e={table.coefficient_of_restitution}\n")

    for step in range(301):
        if step % 30 == 0:
            print(
                f"t={sim.time:6.3f}s  "
                f"pos=({ball.position[0]:.4f}, {ball.position[1]:.4f})  "
                f"vel=({ball.velocity[0]:.4f}, {ball.velocity[1]:.4f})"
            )
        if step < 300:
            sim.step()

    print(f"\nInside playable area: x <= {table.width - r:.4f}")


if __name__ == "__main__":
    main()
