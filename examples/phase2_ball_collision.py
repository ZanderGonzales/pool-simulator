"""Phase 2 demo: two balls collide head-on with no rolling resistance."""

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
    center = vec2(0.5 * table.width, 0.5 * table.height)
    ball_a = Ball(id=0, position=center + vec2(-r - 0.05, 0.0), velocity=vec2(1.5, 0.0))
    ball_b = Ball(id=1, position=center + vec2(r + 0.05, 0.0), velocity=vec2(-1.5, 0.0))
    sim = Simulation(
        balls=[ball_a, ball_b],
        table=table,
        config=SimulationConfig(dt=0.005, collision_iterations=4),
    )

    print("Phase 2 - ball-ball collision")
    print(f"Restitution e={table.coefficient_of_restitution}\n")

    for step in range(101):
        if step % 20 == 0:
            print(
                f"t={sim.time:6.3f}s  "
                f"A: pos=({ball_a.position[0]:.4f}, {ball_a.position[1]:.4f}) "
                f"vel=({ball_a.velocity[0]:.4f}, {ball_a.velocity[1]:.4f})  "
                f"B: pos=({ball_b.position[0]:.4f}, {ball_b.position[1]:.4f}) "
                f"vel=({ball_b.velocity[0]:.4f}, {ball_b.velocity[1]:.4f})"
            )
        if step < 100:
            sim.step()

    print("\nDone.")


if __name__ == "__main__":
    main()
